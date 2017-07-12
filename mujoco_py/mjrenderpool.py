import ctypes
from multiprocessing import (
    Process, set_start_method, freeze_support, Condition,
    Value, Array, Pool, get_logger)
import numpy as np

logger = get_logger()


class RenderPoolStorage:

    __slots__ = ['shared_rgbs_array',
                 'shared_depths_array',
                 'device_id',
                 'worker_id']


class RenderPool:

    DEFAULT_MAX_IMAGE_SIZE = 512 * 512

    def __init__(self, model, device_ids=1,
                 max_image_size=None, n_workers=None):
        if isinstance(device_ids, int):
            device_ids = list(range(device_ids))
        else:
            assert isinstance(device_ids, list), (
                "device_ids must be list of integer")

        if n_workers is None:
            n_workers = len(device_ids)
        self._n_workers = n_workers

        if max_image_size is None:
            max_image_size = RenderPool.DEFAULT_MAX_IMAGE_SIZE
        else:
            assert isinstance(max_image_size, int)
        self._max_image_size = max_image_size

        array_size = self._max_image_size * n_workers

        self._shared_rgbs = Array(ctypes.c_uint, array_size * 3)
        self._shared_depths = Array(ctypes.c_float, array_size)

        self._shared_rgbs_array = np.frombuffer(self._shared_rgbs.get_obj())
        self._shared_depths_array = np.frombuffer(self._shared_depths.get_obj())

        worker_id = Value(ctypes.c_int)
        worker_id.value = 0
        self.pool = Pool(
            processes=n_workers,
            initializer=RenderPool._worker_init,
            initargs=(
                worker_id,
                device_ids,
                self._shared_rgbs,
                self._shared_depths))

    @staticmethod
    def _worker_init(worker_id, device_ids, shared_rgbs, shared_depths):
        logger.info("worker/_worker_init: start")
        s = RenderPoolStorage()

        with worker_id.get_lock():
            s.worker_id = worker_id.value
            worker_id.value += 1
        s.device_id = device_ids[s.worker_id % len(device_ids)]

        s.shared_rgbs_array = np.frombuffer(shared_rgbs.get_obj())
        s.shared_depths_array = np.frombuffer(shared_depths.get_obj())

        logger.info("worker/_worker_init: device_id=%d", s.device_id)
        logger.info("worker/_worker_init: worker_id=%d", s.worker_id)

        global _render_pool_storage
        _render_pool_storage = s
        logger.info("worker/_worker_init: end")

    @staticmethod
    def _worker_render(args):
        width, height, camera_name, depth = args

        s = _render_pool_storage
        logger.info("worker(%d)/_worker_render: %d %d %s %s",
                    s.worker_id,
                    width, height, camera_name, depth)
        logger.info("worker(%d)/_worker_render: device_id=%d",
                    s.worker_id, s.device_id)

        rgb_block = width * height * 3
        rgb_offset = rgb_block * s.worker_id
        rgb = s.shared_rgbs_array[rgb_offset:rgb_offset + rgb_block]
        rgb = rgb.reshape(height, width, 3)
        rgb[..., 0] = s.worker_id * 3
        rgb[..., 1] = s.worker_id * 3 + 1
        rgb[..., 2] = s.worker_id * 3 + 2

    def render(self, width, height, states=None, camera_name=None,
               depth=False):
        if (width * height) > self._max_image_size:
            raise ValueError(
                "Requested image larger than maxiumum image size. Create "
                "a new RenderPool with a larger maximum image size.")

        self.pool.map(
            RenderPool._worker_render,
            [(width, height, camera_name, depth)] * self._n_workers)

        rgbs = self._shared_rgbs_array[:width * height * 3 * self._n_workers]
        rgbs = rgbs.reshape(self._n_workers, height, width, 3)
        return rgbs

    def close(self):
        self.pool.close()
        self.pool.join()
