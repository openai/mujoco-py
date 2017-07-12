import ctypes
from multiprocessing import Array, Pool, Value
import numpy as np

from mujoco_py import MjSim, load_model_from_mjb


class RenderPoolStorage:

    __slots__ = ['shared_rgbs_array',
                 'shared_depths_array',
                 'device_id',
                 'worker_id',
                 'sim']


class RenderPool:

    DEFAULT_MAX_IMAGE_SIZE = 512 * 512

    def __init__(self, model, device_ids=1, n_workers=None,
                 max_batch_size=None, max_image_size=None):
        if isinstance(device_ids, int):
            device_ids = list(range(device_ids))
        else:
            assert isinstance(device_ids, list), (
                "device_ids must be list of integer")

        n_workers = n_workers or 1
        self._max_batch_size = max_batch_size or len(device_ids)

        if max_image_size is None:
            max_image_size = RenderPool.DEFAULT_MAX_IMAGE_SIZE
        else:
            assert isinstance(max_image_size, int)
        self._max_image_size = max_image_size

        array_size = self._max_image_size * self._max_batch_size

        self._shared_rgbs = Array(ctypes.c_uint, array_size * 3)
        self._shared_depths = Array(ctypes.c_float, array_size)

        self._shared_rgbs_array = np.frombuffer(self._shared_rgbs.get_obj())
        self._shared_depths_array = np.frombuffer(self._shared_depths.get_obj())

        worker_id = Value(ctypes.c_int)
        worker_id.value = 0
        self.pool = Pool(
            processes=len(device_ids) * n_workers,
            initializer=RenderPool._worker_init,
            initargs=(
                model.get_mjb(),
                worker_id,
                device_ids,
                self._shared_rgbs,
                self._shared_depths))

    @staticmethod
    def _worker_init(mjb_bytes, worker_id, device_ids,
                     shared_rgbs, shared_depths):
        s = RenderPoolStorage()

        with worker_id.get_lock():
            proc_worker_id = worker_id.value
            worker_id.value += 1
        s.device_id = device_ids[proc_worker_id % len(device_ids)]

        s.shared_rgbs_array = np.frombuffer(shared_rgbs.get_obj())
        s.shared_depths_array = np.frombuffer(shared_depths.get_obj())

        s.sim = MjSim(load_model_from_mjb(mjb_bytes))
        s.sim.render(100, 100, device_id=s.device_id)

        global _render_pool_storage
        _render_pool_storage = s

    @staticmethod
    def _worker_render(worker_id, state, width, height, camera_name):
        s = _render_pool_storage

        if state is not None:
            s.sim.set_state(state)
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
            width, height, camera_name=camera_name, depth=True)

    def render(self, width, height, states=None, camera_name=None,
               depth=False):
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
            RenderPool._worker_render,
            [(i, state, width, height, camera_name)
             for i, state in enumerate(states)])

        rgbs = self._shared_rgbs_array[:width * height * 3 * batch_size]
        rgbs = rgbs.reshape(batch_size, height, width, 3)

        if depth:
            depths = self._shared_depths_array[:width * height * batch_size]
            depths = depths.reshape(batch_size, height, width)
            return rgbs, depths
        else:
            return rgbs

    def close(self):
        self.pool.close()
        self.pool.join()
