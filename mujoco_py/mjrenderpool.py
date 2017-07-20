import ctypes
import inspect

from multiprocessing import Array, get_start_method, Pool, Value

import numpy as np


class RenderPoolStorage:
    """
    Helper object used for storing global data for worker processes.
    """

    __slots__ = ['shared_rgbs_array',
                 'shared_depths_array',
                 'device_id',
                 'sim',
                 'modder']


class MjRenderPool:
    """
    Utilizes a process pool to render a MuJoCo simulation across
    multiple GPU devices. This can scale the throughput linearly
    with the number of available GPUs. Throughput can also be
    slightly increased by using more than one worker per GPU.
    """

    DEFAULT_MAX_IMAGE_SIZE = 512 * 512  # in pixels

    def __init__(self, model, device_ids=1, n_workers=None,
                 max_batch_size=None, max_image_size=DEFAULT_MAX_IMAGE_SIZE,
                 modder=None):
        """
        Args:
        - model (PyMjModel): MuJoCo model to use for rendering
        - device_ids (int/list): list of device ids to use for rendering.
            One or more workers will be assigned to each device, depending
            on how many workers are requested.
        - n_workers (int): number of parallel processes in the pool. Defaults
            to the number of device ids.
        - max_batch_size (int): maximum number of states that can be rendered
            in batch using .render(). Defaults to the number of workers.
        - max_image_size (int): maximum number pixels in images requested
            by .render()
        - modder (Modder): modder to use for domain randomization.
        """
        self._closed, self.pool = False, None

        if not (modder is None or inspect.isclass(modder)):
            raise ValueError("modder must be a class")

        if isinstance(device_ids, int):
            device_ids = list(range(device_ids))
        else:
            assert isinstance(device_ids, list), (
                "device_ids must be list of integer")

        n_workers = n_workers or 1
        self._max_batch_size = max_batch_size or (len(device_ids) * n_workers)
        self._max_image_size = max_image_size

        array_size = self._max_image_size * self._max_batch_size

        self._shared_rgbs = Array(ctypes.c_uint8, array_size * 3)
        self._shared_depths = Array(ctypes.c_float, array_size)

        self._shared_rgbs_array = np.frombuffer(
            self._shared_rgbs.get_obj(), dtype=ctypes.c_uint8)
        assert self._shared_rgbs_array.size == (array_size * 3), (
            "Array size is %d, expected %d" % (
                self._shared_rgbs_array.size, array_size * 3))
        self._shared_depths_array = np.frombuffer(
            self._shared_depths.get_obj(), dtype=ctypes.c_float)
        assert self._shared_depths_array.size == array_size, (
            "Array size is %d, expected %d" % (
                self._shared_depths_array.size, array_size))

        worker_id = Value(ctypes.c_int)
        worker_id.value = 0

        if get_start_method() != "spawn":
            raise RuntimeError(
                "Start method must be set to 'spawn' for the "
                "render pool to work. That is, you must add the "
                "following to the _TOP_ of your main script, "
                "before any other imports (since they might be "
                "setting it otherwise):\n"
                "  import multiprocessing as mp\n"
                "  if __name__ == '__main__':\n"
                "    mp.set_start_method('spawn')\n")

        self.pool = Pool(
            processes=len(device_ids) * n_workers,
            initializer=MjRenderPool._worker_init,
            initargs=(
                model.get_mjb(),
                worker_id,
                device_ids,
                self._shared_rgbs,
                self._shared_depths,
                modder))

    @staticmethod
    def _worker_init(mjb_bytes, worker_id, device_ids,
                     shared_rgbs, shared_depths, modder):
        """
        Initializes the global state for the workers.
        """
        s = RenderPoolStorage()

        with worker_id.get_lock():
            proc_worker_id = worker_id.value
            worker_id.value += 1
        s.device_id = device_ids[proc_worker_id % len(device_ids)]

        s.shared_rgbs_array = np.frombuffer(
            shared_rgbs.get_obj(), dtype=ctypes.c_uint8)
        s.shared_depths_array = np.frombuffer(
            shared_depths.get_obj(), dtype=ctypes.c_float)

        # avoid a circular import
        from mujoco_py import load_model_from_mjb, MjRenderContext, MjSim
        s.sim = MjSim(load_model_from_mjb(mjb_bytes))
        # attach a render context to the sim (needs to happen before
        # modder is called, since it might need to upload textures
        # to the GPU).
        MjRenderContext(s.sim, device_id=s.device_id)

        if modder is not None:
            s.modder = modder(s.sim, random_state=proc_worker_id)
            s.modder.whiten_materials()
        else:
            s.modder = None

        global _render_pool_storage
        _render_pool_storage = s

    @staticmethod
    def _worker_render(worker_id, state, width, height,
                       camera_name, randomize):
        """
        Main target function for the workers.
        """
        s = _render_pool_storage

        forward = False
        if state is not None:
            s.sim.set_state(state)
            forward = True
        if randomize and s.modder is not None:
            s.modder.randomize()
            forward = True
        if forward:
            s.sim.forward()

        rgb_block = width * height * 3
        rgb_offset = rgb_block * worker_id
        rgb = s.shared_rgbs_array[rgb_offset:rgb_offset + rgb_block]
        rgb = rgb.reshape(height, width, 3)

        depth_block = width * height
        depth_offset = depth_block * worker_id
        depth = s.shared_depths_array[depth_offset:depth_offset + depth_block]
        depth = depth.reshape(height, width)

        rgb[:], depth[:] = s.sim.render(
            width, height, camera_name=camera_name, depth=True,
            device_id=s.device_id)

    def render(self, width, height, states=None, camera_name=None,
               depth=False, randomize=False, copy=True):
        """
        Renders the simulations in batch. If no states are provided,
        the max_batch_size will be used.

        Args:
        - width (int): width of image to render.
        - height (int): height of image to render.
        - states (list): list of MjSimStates; updates the states before
            rendering. Batch size will be number of states supplied.
        - camera_name (str): name of camera to render from.
        - depth (bool): if True, also return depth.
        - randomize (bool): calls modder.rand_all() before rendering.
        - copy (bool): return a copy rather than a reference

        Returns:
        - rgbs: NxHxWx3 numpy array of N images in batch of width W
            and height H.
        - depth: NxHxW numpy array of N images in batch of width W
            and height H. Only returned if depth=True.
        """
        if self._closed:
            raise RuntimeError("The pool has been closed.")

        if (width * height) > self._max_image_size:
            raise ValueError(
                "Requested image larger than maximum image size. Create "
                "a new RenderPool with a larger maximum image size.")
        if states is None:
            batch_size = self._max_batch_size
            states = [None] * batch_size
        else:
            batch_size = len(states)

        if batch_size > self._max_batch_size:
            raise ValueError(
                "Requested batch size larger than max batch size. Create "
                "a new RenderPool with a larger max batch size.")

        self.pool.starmap(
            MjRenderPool._worker_render,
            [(i, state, width, height, camera_name, randomize)
             for i, state in enumerate(states)])

        rgbs = self._shared_rgbs_array[:width * height * 3 * batch_size]
        rgbs = rgbs.reshape(batch_size, height, width, 3)
        if copy:
            rgbs = rgbs.copy()

        if depth:
            depths = self._shared_depths_array[:width * height * batch_size]
            depths = depths.reshape(batch_size, height, width).copy()
            if copy:
                depths = depths.copy()
            return rgbs, depths
        else:
            return rgbs

    def close(self):
        """
        Closes the pool and terminates child processes.
        """
        if not self._closed:
            if self.pool is not None:
                self.pool.close()
                self.pool.join()
            self._closed = True

    def __del__(self):
        self.close()
