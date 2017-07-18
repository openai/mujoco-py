class MjBatchRenderer(object):

    def __init__(self, sim, width, height, batch_size=1, device_id=0, depth=False):
        # print("XXX Creating render context", flush=True)
        self.render_context = MjRenderContext(sim, device_id=device_id)
        # print("XXX Updating render context", flush=True)
        self.render_context.update_offscreen_size(width, height)
        # print("XXX Creating PBO", flush=True)
        self.pbo = createPBO(width, height, batch_size)

        self._width = width
        self._height = height
        self._batch_size = batch_size
        self._current_batch_offset = 0
        # print("XXX Done initializing MjBatchRenderer", flush=True)

    def render(self, camera_id=None, batch_offset=None):
        # print("XXX render", flush=True)

        if batch_offset is not None:
            if batch_offset < 0 or batch_offset >= self._batch_size:
                raise ValueError("batch_offset out of range")
            self._current_batch_offset = batch_offset

        # print("XXX render.render", flush=True)
        self.render_context.render(self._width, self._height, camera_id=camera_id)

        cdef mjrRect viewport
        viewport.left = 0
        viewport.bottom = 0
        viewport.width = self._width
        viewport.height = self._height

        # print("XXX reference context", flush=True)
        cdef PyMjrContext con = <PyMjrContext> self.render_context.con

        # print("XXX call copyFBOToPBO", flush=True)
        copyFBOToPBO(con.ptr, self.pbo, viewport, self._current_batch_offset)

        self._current_batch_offset = (self._current_batch_offset + 1) % self._batch_size
        # print("XXX done render", flush=True)

    def read(self):
        # print("XXX read", flush=True)
        rgb_arr = np.zeros(3 * self._width * self._height * self._batch_size, dtype=np.uint8)
        cdef unsigned char[::view.contiguous] rgb_view = rgb_arr
        # print("XXX read readPBO", flush=True)
        readPBO(&rgb_view[0], self.pbo, self._width, self._height, self._batch_size)
        # print("XXX read readPBO done", flush=True)
        return rgb_arr.reshape(self._batch_size,
                               self._height,
                               self._width,
                               3)

    def __del__(self):
        freePBO(self.pbo)
