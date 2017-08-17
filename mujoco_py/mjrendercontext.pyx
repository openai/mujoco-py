from threading import Lock
from mujoco_py.generated import const

cdef class MjRenderContext(object):
    """
    Class that encapsulates rendering functionality for a
    MuJoCo simulation.
    """

    cdef mjModel *_model_ptr
    cdef mjData *_data_ptr

    cdef mjvScene _scn
    cdef mjvCamera _cam
    cdef mjvOption _vopt
    cdef mjvPerturb _pert
    cdef mjrContext _con

    # Public wrappers
    cdef readonly PyMjvScene scn
    cdef readonly PyMjvCamera cam
    cdef readonly PyMjvOption vopt
    cdef readonly PyMjvPerturb pert
    cdef readonly PyMjrContext con

    cdef readonly object opengl_context
    cdef readonly int _visible
    cdef readonly list _markers
    cdef readonly dict _overlay

    cdef readonly bint offscreen
    cdef public object sim

    def __cinit__(self):
        maxgeom = 1000
        mjv_makeScene(&self._scn, maxgeom)
        mjv_defaultCamera(&self._cam)
        mjv_defaultOption(&self._vopt)
        mjr_defaultContext(&self._con)

    def __init__(self, MjSim sim, bint offscreen=True, int device_id=-1):
        self.sim = sim
        self._setup_opengl_context(offscreen, device_id)
        self.offscreen = offscreen

        # Ensure the model data has been updated so that there
        # is something to render
        sim.forward()

        sim.add_render_context(self)

        self._model_ptr = sim.model.ptr
        self._data_ptr = sim.data.ptr
        self.scn = WrapMjvScene(&self._scn)
        self.cam = WrapMjvCamera(&self._cam)
        self.vopt = WrapMjvOption(&self._vopt)
        self.con = WrapMjrContext(&self._con)
        self._pert.active = 0
        self._pert.select = 0
        self.pert = WrapMjvPerturb(&self._pert)

        self._markers = []
        self._overlay = {}

        self._init_camera(sim)
        self._set_mujoco_buffers()

    def update_sim(self, MjSim new_sim):
        if new_sim == self.sim:
            return
        self._model_ptr = new_sim.model.ptr
        self._data_ptr = new_sim.data.ptr
        self._set_mujoco_buffers()
        for render_context in self.sim.render_contexts:
            new_sim.add_render_context(render_context)
        self.sim = new_sim

    def _set_mujoco_buffers(self):
        mjr_makeContext(self._model_ptr, &self._con, mjFONTSCALE_150)
        if self.offscreen:
            mjr_setBuffer(mjFB_OFFSCREEN, &self._con);
            if self._con.currentBuffer != mjFB_OFFSCREEN:
                raise RuntimeError('Offscreen rendering not supported')
        else:
            mjr_setBuffer(mjFB_WINDOW, &self._con);
            if self._con.currentBuffer != mjFB_WINDOW:
                raise RuntimeError('Window rendering not supported')
        self.con = WrapMjrContext(&self._con)

    def _setup_opengl_context(self, offscreen, device_id):
        if not offscreen or sys.platform == 'darwin':
            self.opengl_context = GlfwContext(offscreen=offscreen)
        else:
            if device_id < 0:
                if "GPUS" in os.environ:
                    device_id = int(os.environ["GPUS"].split(',')[0])
                else:
                    device_id = int(os.getenv('CUDA_VISIBLE_DEVICES', '0').split(',')[0])
            self.opengl_context = OffscreenOpenGLContext(device_id)

    def _init_camera(self, sim):
        # Make the free camera look at the scene
        self.cam.type = const.CAMERA_FREE
        self.cam.fixedcamid = -1
        for i in range(3):
            self.cam.lookat[i] = sim.model.stat.center[i]
        self.cam.distance = sim.model.stat.extent

    def update_offscreen_size(self, width, height):
        if width != self._con.offWidth or height != self._con.offHeight:
            self._model_ptr.vis.global_.offwidth = width
            self._model_ptr.vis.global_.offheight = height
            mjr_freeContext(&self._con)
            self._set_mujoco_buffers()

    def render(self, width, height, camera_id=None):
        cdef mjrRect rect
        rect.left = 0
        rect.bottom = 0
        rect.width = width
        rect.height = height

        # Sometimes buffers are too small.
        if width > self._con.offWidth or height > self._con.offHeight:
            new_width = max(width, self._model_ptr.vis.global_.offwidth)
            new_height = max(height, self._model_ptr.vis.global_.offheight)
            self.update_offscreen_size(new_width, new_height)

        if camera_id is not None:
            if camera_id == -1:
                self.cam.type = const.CAMERA_FREE
            else:
                self.cam.type = const.CAMERA_FIXED
            self.cam.fixedcamid = camera_id

        self.opengl_context.set_buffer_size(width, height)

        mjv_updateScene(self._model_ptr, self._data_ptr, &self._vopt,
                        &self._pert, &self._cam, mjCAT_ALL, &self._scn)

        for marker_params in self._markers:
            self._add_marker_to_scene(marker_params)

        mjr_render(rect, &self._scn, &self._con)
        for gridpos, (text1, text2) in self._overlay.items():
            mjr_overlay(const.FONTSCALE_150, gridpos, rect, text1.encode(), text2.encode(), &self._con)

    def read_pixels(self, width, height, depth=True):
        cdef mjrRect rect
        rect.left = 0
        rect.bottom = 0
        rect.width = width
        rect.height = height

        rgb_arr = np.zeros(3 * rect.width * rect.height, dtype=np.uint8)
        depth_arr = np.zeros(rect.width * rect.height, dtype=np.float32)
        cdef unsigned char[::view.contiguous] rgb_view = rgb_arr
        cdef float[::view.contiguous] depth_view = depth_arr
        mjr_readPixels(&rgb_view[0], &depth_view[0], rect, &self._con)
        rgb_img = np.flipud(rgb_arr.reshape(rect.height, rect.width, 3))
        if depth:
            depth_img = np.flipud(depth_arr.reshape(rect.height, rect.width))
            return (rgb_img, depth_img)
        else:
            return rgb_img

    def upload_texture(self, int tex_id):
        """ Uploads given texture to the GPU. """
        self.opengl_context.make_context_current()
        mjr_uploadTexture(self._model_ptr, &self._con, tex_id)

    def draw_pixels(self, np.ndarray[np.uint8_t, ndim=3] image, int left, int bottom):
        """Draw an image into the OpenGL buffer."""
        cdef unsigned char[::view.contiguous] image_view = image.ravel()
        cdef mjrRect viewport
        viewport.left = left
        viewport.bottom = bottom
        viewport.width = image.shape[1]
        viewport.height = image.shape[0]
        mjr_drawPixels(&image_view[0], NULL, viewport, &self._con)

    def move_camera(self, int action, double reldx, double reldy):
        """ Moves the camera based on mouse movements. Action is one of mjMOUSE_*. """
        mjv_moveCamera(self._model_ptr, action, reldx, reldy, &self._scn, &self._cam)

    def add_overlay(self, int gridpos, str text1, str text2):
        """ Overlays text on the scene. """
        if gridpos not in self._overlay:
            self._overlay[gridpos] = ["", ""]
        self._overlay[gridpos][0] += text1 + "\n"
        self._overlay[gridpos][1] += text2 + "\n"

    def add_marker(self, **marker_params):
        self._markers.append(marker_params)

    def _add_marker_to_scene(self, marker_params):
        """ Adds marker to scene, and returns the corresponding object. """
        if self._scn.ngeom >= self._scn.maxgeom:
            raise RuntimeError('Ran out of geoms. maxgeom: %d' % self._scn.maxgeom)

        cdef mjvGeom *g = self._scn.geoms + self._scn.ngeom

        # default values.
        g.dataid = -1
        g.objtype = const.OBJ_UNKNOWN
        g.objid = -1
        g.category = const.CAT_DECOR
        g.texid = -1
        g.texuniform = 0
        g.texrepeat[0] = 1
        g.texrepeat[1] = 1
        g.emission = 0
        g.specular = 0.5
        g.shininess = 0.5
        g.reflectance = 0
        g.type = const.GEOM_BOX
        g.size[:] = np.ones(3) * 0.1
        g.mat[:] = np.eye(3).flatten()
        g.rgba[:] = np.ones(4)
        wrapped = WrapMjvGeom(g)

        for key, value in marker_params.items():
            if isinstance(value, (int, float)):
                setattr(wrapped, key, value)
            elif isinstance(value, (tuple, list, np.ndarray)):
                attr = getattr(wrapped, key)
                attr[:] = np.asarray(value).reshape(attr.shape)
            elif isinstance(value, str):
                assert key == "label", "Only label is a string in mjvGeom."
                if value == None:
                    g.label[0] = 0
                else:
                    strncpy(g.label, value.encode(), 100)
            elif hasattr(wrapped, key):
                raise ValueError("mjvGeom has attr {} but type {} is invalid".format(key, type(value)))
            else:
                raise ValueError("mjvGeom doesn't have field %s" % key)

        self._scn.ngeom += 1


    def __dealloc__(self):
        mjr_freeContext(&self._con)
        mjv_freeScene(&self._scn)


class MjRenderContextOffscreen(MjRenderContext):

    def __cinit__(self, MjSim sim, int device_id):
        super().__init__(sim, offscreen=True, device_id=device_id)

class MjRenderContextWindow(MjRenderContext):

    def __init__(self, MjSim sim):
        super().__init__(sim, offscreen=False)

        assert isinstance(self.opengl_context, GlfwContext), (
            "Only GlfwContext supported for windowed rendering")

    @property
    def window(self):
        return self.opengl_context.window

    def render(self):
        if self.window is None or glfw.window_should_close(self.window):
            return

        glfw.make_context_current(self.window)
        super().render(*glfw.get_framebuffer_size(self.window))
        glfw.swap_buffers(self.window)
