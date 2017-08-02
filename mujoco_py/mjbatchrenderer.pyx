try:
    import pycuda.driver as drv
except ImportError:
    drv = None


class MjBatchRendererException(Exception):
    pass


class MjBatchRendererNotSupported(MjBatchRendererException):
    pass


class CudaNotEnabledError(MjBatchRendererException):
    pass


class CudaBufferNotMappedError(MjBatchRendererException):
    pass


class CudaBufferMappedError(MjBatchRendererException):
    pass


class MjBatchRenderer(object):
    """
    Utility class for rendering into OpenGL Pixel Buffer Objects (PBOs),
    which allows for accessing multiple rendered images in batch.

    If used with CUDA (i.e. initialized with use_cuda=True), you need
    to call map/unmap when accessing CUDA buffer pointer. This is to
    ensure that all OpenGL instructions have completed:

        renderer = MjBatchRenderer(100, 100, use_cuda=True)
        renderer.render(sim)

        renderer.map()
        image = renderer.read()
        renderer.unmap()
    """

    def __init__(self, width, height, batch_size=1, device_id=0,
                 depth=False, use_cuda=False):
        """
        Args:
        - width (int): Image width.
        - height (int): Image height.
        - batch_size (int): Size of batch to render into. Memory is
            allocated once upon initialization of object.
        - device_id (int): Device to use for storing the batch.
        - depth (bool): if True, render depth in addition to RGB.
        - use_cuda (bool): if True, use OpenGL-CUDA interop to map
            the PBO onto a CUDA buffer.
        """
        # Early initialization to prevent failure in __del__
        self._use_cuda = False
        self.pbo_depth, self.pbo_depth = 0, 0

        if not usingEGL():
            raise MjBatchRendererNotSupported(
                "MjBatchRenderer currently only supported with EGL-backed"
                "rendering context.")

        # Make sure OpenGL Context is available before creating PBOs
        initOpenGL(device_id)
        makeOpenGLContextCurrent(device_id)

        self.pbo_rgb = createPBO(width, height, batch_size, 0)
        self.pbo_depth = createPBO(width, height, batch_size, 1) if depth else 0

        self._depth = depth
        self._device_id = device_id
        self._width = width
        self._height = height
        self._batch_size = batch_size
        self._current_batch_offset = 0

        self._use_cuda = use_cuda
        self._cuda_buffers_are_mapped = False
        self._cuda_rgb_ptr, self._cuda_depth_ptr = None, None
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
        if self._depth:
            self._cuda_depth_pbo = RegisteredBuffer(self.pbo_depth)

    def map(self):
        """ Map OpenGL buffer to CUDA for reading. """
        if not self._use_cuda:
            raise CudaNotEnabledError()
        elif self._cuda_buffers_are_mapped:
            return  # just make it a no-op

        self._cuda_context.push()
        self._cuda_rgb_mapping = self._cuda_rgb_pbo.map()
        ptr, self._cuda_rgb_buf_size = (
            self._cuda_rgb_mapping.device_ptr_and_size())
        assert ptr is not None and self._cuda_rgb_buf_size > 0
        if self._cuda_rgb_ptr is None:
            self._cuda_rgb_ptr = ptr

        # There doesn't seem to be a guarantee from the API that the
        # pointer will be the same between mappings, but empirically
        # this has been true. If this isn't true, we need to modify
        # the interface to MjBatchRenderer to make this clearer to user.
        # So, hopefully we won't hit this assert.
        assert self._cuda_rgb_ptr == ptr, (
            "Mapped CUDA rgb buffer pointer %d doesn't match old pointer %d" %
            (ptr, self._cuda_rgb_ptr))

        if self._depth:
            self._cuda_depth_mapping = self._cuda_depth_pbo.map()
            ptr, self._cuda_depth_buf_size = (
                self._cuda_depth_mapping.device_ptr_and_size())
            assert ptr is not None and self._cuda_depth_buf_size > 0
            if self._cuda_depth_ptr is None:
                self._cuda_depth_ptr = ptr
            assert self._cuda_depth_ptr == ptr, (
                "Mapped CUDA depth buffer pointer %d doesn't match old pointer %d" %
                (ptr, self._cuda_depth_ptr))

        self._cuda_buffers_are_mapped = True

    def unmap(self):
        """ Unmap OpenGL buffer from CUDA so that it can be rendered into. """
        if not self._use_cuda:
            raise CudaNotEnabledError()
        elif not self._cuda_buffers_are_mapped:
            return  # just make it a no-op

        self._cuda_context.push()
        self._cuda_rgb_mapping.unmap()
        self._cuda_rgb_mapping = None
        self._cuda_rgb_ptr = None
        if self._depth:
            self._cuda_depth_mapping.unmap()
            self._cuda_depth_mapping = None
            self._cuda_depth_ptr = None

        self._cuda_buffers_are_mapped = False

    def prepare_render_context(self, sim):
        """
        Set up the rendering context for an MjSim. Also happens automatically
        on `.render()`.
        """
        for c in sim.render_contexts:
            if (c.offscreen and
                    isinstance(c.opengl_context, OffscreenOpenGLContext) and
                    c.opengl_context.device_id == self._device_id):
                return c

        return MjRenderContext(sim, device_id=self._device_id)

    def render(self, sim, camera_id=None, batch_offset=None):
        """
        Render current scene from the MjSim into the buffer. By
        default the batch offset is automatically incremented with
        each call. It can be reset with the batch_offset parameter.

        This method doesn't return anything. Use the `.read` method
        to read the buffer, or access the buffer pointer directly with
        e.g. `.cuda_rgb_buffer_pointer` accessor.

        Args:
        - sim (MjSim): The simulator to use for rendering.
        - camera_id (int): MuJoCo id for the camera, from
            `sim.model.camera_name2id()`.
        - batch_offset (int): offset in batch to render to.
        """
        if self._use_cuda and self._cuda_buffers_are_mapped:
            raise CudaBufferMappedError(
                "CUDA buffers must be unmapped before calling render.")

        if batch_offset is not None:
            if batch_offset < 0 or batch_offset >= self._batch_size:
                raise ValueError("batch_offset out of range")
            self._current_batch_offset = batch_offset

        # Ensure the correct device context is used (this takes ~1 µs)
        makeOpenGLContextCurrent(self._device_id)

        render_context = self.prepare_render_context(sim)
        render_context.update_offscreen_size(self._width, self._height)
        render_context.render(self._width, self._height, camera_id=camera_id)

        cdef mjrRect viewport
        viewport.left = 0
        viewport.bottom = 0
        viewport.width = self._width
        viewport.height = self._height

        cdef PyMjrContext con = <PyMjrContext> render_context.con
        copyFBOToPBO(con.ptr, self.pbo_rgb, self.pbo_depth,
                     viewport, self._current_batch_offset)

        self._current_batch_offset = (self._current_batch_offset + 1) % self._batch_size

    def read(self):
        """
        Transfer a copy of the buffer from the GPU to the CPU as a numpy array.

        Returns:
        - rgb_batch (numpy array): batch of rgb images in uint8 NHWC format
        - depth_batch (numpy array): batch of depth images in uint16 NHWC format
        """
        if self._use_cuda:
            return self._read_cuda()
        else:
            return self._read_nocuda()

    def _read_cuda(self):
        if not self._cuda_buffers_are_mapped:
            raise CudaBufferNotMappedError(
                "CUDA buffers must be mapped before reading")

        rgb_arr = drv.from_device(
            self._cuda_rgb_ptr,
            shape=(self._batch_size, self._height, self._width, 3),
            dtype=np.uint8)

        if self._depth:
            depth_arr = drv.from_device(
                self._cuda_depth_ptr,
                shape=(self._batch_size, self._height, self._width),
                dtype=np.uint16)
        else:
            depth_arr = None

        return rgb_arr, depth_arr

    def _read_nocuda(self):
        rgb_arr = np.zeros(3 * self._width * self._height * self._batch_size, dtype=np.uint8)
        cdef unsigned char[::view.contiguous] rgb_view = rgb_arr
        depth_arr = np.zeros(self._width * self._height * self._batch_size, dtype=np.uint16)
        cdef unsigned short[::view.contiguous] depth_view = depth_arr

        if self._depth:
            readPBO(&rgb_view[0], &depth_view[0], self.pbo_rgb, self.pbo_depth,
                    self._width, self._height, self._batch_size)
            depth_arr = depth_arr.reshape(self._batch_size, self._height, self._width)
        else:
            readPBO(&rgb_view[0], NULL, self.pbo_rgb, 0,
                    self._width, self._height, self._batch_size)
            # Fine to throw aray depth_arr above since malloc/free is cheap
            depth_arr = None

        rgb_arr = rgb_arr.reshape(self._batch_size, self._height, self._width, 3)
        return rgb_arr, depth_arr

    @property
    def cuda_rgb_buffer_pointer(self):
        """ Pointer to CUDA buffer for RGB batch. """
        if not self._use_cuda:
            raise CudaNotEnabledError()
        elif not self._cuda_buffers_are_mapped:
            raise CudaBufferNotMappedError()
        return self._cuda_rgb_ptr

    @property
    def cuda_depth_buffer_pointer(self):
        """ Pointer to CUDA buffer for depth batch. """
        if not self._use_cuda:
            raise CudaNotEnabledError()
        elif not self._cuda_buffers_are_mapped:
            raise CudaBufferNotMappedError()
        if not self._depth:
            raise RuntimeError("Depth not enabled. Use depth=True on initialization.")
        return self._cuda_depth_ptr

    def __del__(self):
        if self._use_cuda:
            self._cuda_context.push()
            self.unmap()
            self._cuda_rgb_pbo.unregister()
            if self._depth:
                self._cuda_depth_pbo.unregister()

            # Clean up context
            drv.Context.pop()
            self._cuda_context.detach()

        if self.pbo_depth:
            freePBO(self.pbo_rgb)
        if self.pbo_depth:
            freePBO(self.pbo_depth)
