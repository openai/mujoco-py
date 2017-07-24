from ctypes import c_float, sizeof
try:
    import pycuda.driver as drv
except ImportError:
    drv = None


class MjBatchRenderer(object):

    def __init__(self, sim, width, height, batch_size=1, device_id=0,
                 depth=False, use_cuda=False):
        # Early initialization to prevent failure in __del__
        self._use_cuda = False
        self.pbo_depth, self.pbo_depth = 0, 0

        self.render_context = MjRenderContext(sim, device_id=device_id)
        self.render_context.update_offscreen_size(width, height)
        self.pbo_rgb = createPBO(width, height, batch_size, 0)
        self.pbo_depth = createPBO(width, height, batch_size, 1) if depth else 0

        self._depth = depth
        self._device_id = device_id
        self._width = width
        self._height = height
        self._batch_size = batch_size
        self._current_batch_offset = 0

        self._use_cuda = use_cuda
        if use_cuda:
            self._init_cuda()

    def _init_cuda(self):
        if drv is None:
            raise ImportError("Failed to import pycuda.")
        # Use local imports so that we don't have to make pycuda
        # opengl interop a requirement
        from pycuda.gl import RegisteredBuffer

        drv.init()
        device = drv.Device(self._device_id)
        self._cuda_context = device.make_context()
        self._cuda_context.push()

        self._cuda_rgb_pbo = RegisteredBuffer(self.pbo_rgb)
        self._cuda_rgb_mapping = self._cuda_rgb_pbo.map()
        self._cuda_rgb_ptr, self._cuda_rgb_buf_size = (
            self._cuda_rgb_mapping.device_ptr_and_size())

        self._cuda_rgb_buffer = drv.mem_alloc(self._cuda_rgb_buf_size)

        if self._depth:
            self._cuda_depth_pbo = RegisteredBuffer(self.pbo_depth)
            self._cuda_depth_mapping = self._cuda_depth_pbo.map()
            self._cuda_depth_ptr, self._cuda_depth_buf_size = (
                self._cuda_depth_mapping.device_ptr_and_size())

            self._cuda_depth_buffer = drv.mem_alloc(self._cuda_depth_buf_size)

    def render(self, camera_id=None, batch_offset=None):
        if batch_offset is not None:
            if batch_offset < 0 or batch_offset >= self._batch_size:
                raise ValueError("batch_offset out of range")
            self._current_batch_offset = batch_offset

        self.render_context.render(self._width, self._height, camera_id=camera_id)

        cdef mjrRect viewport
        viewport.left = 0
        viewport.bottom = 0
        viewport.width = self._width
        viewport.height = self._height

        cdef PyMjrContext con = <PyMjrContext> self.render_context.con

        copyFBOToPBO(con.ptr, self.pbo_rgb, self.pbo_depth,
                     viewport, self._current_batch_offset)

        self._current_batch_offset = (self._current_batch_offset + 1) % self._batch_size

    def read(self):
        rgb_arr = np.zeros(3 * self._width * self._height * self._batch_size, dtype=np.uint8)

        cdef unsigned char[::view.contiguous] rgb_view = rgb_arr
        cdef float[::view.contiguous] depth_view

        if self._depth:
            depth_arr = np.zeros(self._width * self._height * self._batch_size, dtype=np.float)
            depth_view = depth_arr
            readPBO(&rgb_view[0], &depth_view[0], self.pbo_rgb, self.pbo_depth,
                    self._width, self._height, self._batch_size)
            depth_arr = depth_arr.reshape(self._batch_size, self._height, self._width)
        else:
            readPBO(&rgb_view[0], NULL, self.pbo_rgb, self.pbo_depth,
                    self._width, self._height, self._batch_size)
            depth_arr = None

        rgb_arr = rgb_arr.reshape(self._batch_size, self._height, self._width, 3)
        return rgb_arr, depth_arr

    def copy_gpu_buffers(self):
        drv.memcpy_dtod(self._cuda_rgb_buffer, self._cuda_rgb_ptr, self._cuda_rgb_buf_size)
        if self._depth:
            drv.memcpy_dtod(self._cuda_depth_buffer, self._cuda_depth_ptr, self._cuda_depth_buf_size)

    @property
    def cuda_rgb_buffer_pointer(self):
        return self._cuda_rgb_ptr

    @property
    def cuda_depth_buffer_pointer(self):
        if not self._depth:
            raise RuntimeError("Depth not enabled. Use depth=True on initialization.")
        return self._cuda_depth_ptr

    def __del__(self):
        if self._use_cuda:
            self._cuda_context.push()
            self._cuda_rgb_mapping.unmap()
            self._cuda_rgb_pbo.unregister()
            self._cuda_depth_mapping.unmap()
            self._cuda_depth_pbo.unregister()

            # Clean up context
            from pycuda.driver import Context
            Context.pop()
            self._cuda_context.detach()

        if self.pbo_depth:
            freePBO(self.pbo_rgb)
        if self.pbo_depth:
            freePBO(self.pbo_depth)
