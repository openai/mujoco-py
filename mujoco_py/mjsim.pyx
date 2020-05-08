from xml.dom import minidom
from mujoco_py.utils import remove_empty_lines
from mujoco_py.builder import build_callback_fn
from threading import Lock

_MjSim_render_lock = Lock()

ctypedef void (*substep_udd_t)(const mjModel* m, mjData* d)


cdef class MjSim(object):
    """MjSim represents a running simulation including its state.

    Similar to Gym's ``MujocoEnv``, it internally wraps a :class:`.PyMjModel`
    and a :class:`.PyMjData`.

    Parameters
    ----------
    model : :class:`.PyMjModel`
        The model to simulate.
    data : :class:`.PyMjData`
        Optional container for the simulation state. Will be created if ``None``.
    nsubsteps : int
        Optional number of MuJoCo steps to run for every call to :meth:`.step`.
        Buffers will be swapped only once per step.
    udd_callback : fn(:class:`.MjSim`) -> dict
        Optional callback for user-defined dynamics. At every call to
        :meth:`.step`, it receives an MjSim object ``sim`` containing the
        current user-defined dynamics state in ``sim.udd_state``, and returns the
        next ``udd_state`` after applying the user-defined dynamics. This is
        useful e.g. for reward functions that operate over functions of historical
        state.
    substep_callback : str or int or None
        This uses a compiled C function as user-defined dynamics in substeps.
        If given as a string, it's compiled as a C function and set as pointer.
        If given as int, it's interpreted as a function pointer.
        See :meth:`.set_substep_callback` for detailed info.
    userdata_names : list of strings or None
        This is a convenience parameter which is just set on the model.
        Equivalent to calling ``model.set_userdata_names``
    render_callback : callback for rendering.
    """
    # MjRenderContext for rendering camera views.
    cdef readonly list render_contexts
    cdef readonly object _render_context_window
    cdef readonly object _render_context_offscreen

    # MuJoCo model
    cdef readonly PyMjModel model
    # MuJoCo data
    """
    DATAZ
    """
    cdef readonly PyMjData data
    # Number of substeps when calling .step
    cdef public int nsubsteps
    # User defined state.
    cdef public dict udd_state
    # User defined dynamics callback
    cdef readonly object _udd_callback
    # Allows to store extra information in MjSim.
    cdef readonly dict extras
    # Function pointer for substep callback, stored as uintptr
    cdef readonly uintptr_t substep_callback_ptr
    # Callback executed before rendering.
    cdef public object render_callback

    def __cinit__(self, PyMjModel model, PyMjData data=None, int nsubsteps=1,
                  udd_callback=None, substep_callback=None, userdata_names=None,
                  render_callback=None):
        self.nsubsteps = nsubsteps
        self.model = model
        if data is None:
            with wrap_mujoco_warning():
                _data = mj_makeData(self.model.ptr)
            if _data == NULL:
                raise Exception('mj_makeData failed!')
            self.data = WrapMjData(_data, self.model)
        else:
            self.data = data

        self.render_contexts = []
        self._render_context_offscreen = None
        self._render_context_window = None
        self.udd_state = None
        self.udd_callback = udd_callback
        self.render_callback = render_callback
        self.extras = {}
        self.set_substep_callback(substep_callback, userdata_names)

    def reset(self):
        """
        Resets the simulation data and clears buffers.
        """
        with wrap_mujoco_warning():
            mj_resetData(self.model.ptr, self.data.ptr)

        self.udd_state = None
        self.step_udd()

    def forward(self):
        """
        Computes the forward kinematics. Calls ``mj_forward`` internally.
        """
        with wrap_mujoco_warning():
            mj_forward(self.model.ptr, self.data.ptr)

    def set_constants(self):
        """
        Set constant fields of mjModel, corresponding to qpos0 configuration.
        """
        with wrap_mujoco_warning():
            mj_setConst(self.model.ptr, self.data.ptr)

    def step(self, with_udd=True):
        """
        Advances the simulation by calling ``mj_step``.

        If ``qpos`` or ``qvel`` have been modified directly, the user is required to call
        :meth:`.forward` before :meth:`.step` if their ``udd_callback`` requires access to MuJoCo state
        set during the forward dynamics.
        """
        if with_udd:
            self.step_udd()

        with wrap_mujoco_warning():
            for _ in range(self.nsubsteps):
                self.substep_callback()
                mj_step(self.model.ptr, self.data.ptr)

    def render(self, width=None, height=None, *, camera_name=None, depth=False,
               mode='offscreen', device_id=-1, segmentation=False):
        """
        Renders view from a camera and returns image as an `numpy.ndarray`.

        Args:
        - width (int): desired image width.
        - height (int): desired image height.
        - camera_name (str): name of camera in model. If None, the free
            camera will be used.
        - depth (bool): if True, also return depth buffer
        - device (int): device to use for rendering (only for GPU-backed
            rendering).

        Returns:
        - rgb (uint8 array): image buffer from camera
        - depth (float array): depth buffer from camera (only returned
            if depth=True)
        """
        if camera_name is None:
            camera_id = None
        else:
            camera_id = self.model.camera_name2id(camera_name)

        if mode == 'offscreen':
            with _MjSim_render_lock:
                if self._render_context_offscreen is None:
                    render_context = MjRenderContextOffscreen(
                        self, device_id=device_id)
                else:
                    render_context = self._render_context_offscreen

                render_context.render(
                    width=width, height=height, camera_id=camera_id, segmentation=segmentation)
                return render_context.read_pixels(
                    width, height, depth=depth, segmentation=segmentation)
        elif mode == 'window':
            if self._render_context_window is None:
                from mujoco_py.mjviewer import MjViewer
                render_context = MjViewer(self)
            else:
                render_context = self._render_context_window

            render_context.render()

        else:
            raise ValueError("Mode must be either 'window' or 'offscreen'.")

    def add_render_context(self, render_context):
        self.render_contexts.append(render_context)
        if render_context.offscreen and self._render_context_offscreen is None:
            self._render_context_offscreen = render_context
        elif not render_context.offscreen and self._render_context_window is None:
            self._render_context_window = render_context

    @property
    def udd_callback(self):
        return self._udd_callback

    @udd_callback.setter
    def udd_callback(self, value):
        self._udd_callback = value
        self.udd_state = None
        self.step_udd()

    cpdef substep_callback(self):
        if self.substep_callback_ptr:
            (<mjfGeneric>self.substep_callback_ptr)(self.model.ptr, self.data.ptr)

    def set_substep_callback(self, substep_callback, userdata_names=None):
        '''
        Set a substep callback function.

        Parameters :
            substep_callback : str or int or None
                If `substep_callback` is a string, compile to function pointer and set.
                    See `builder.build_callback_fn()` for documentation.
                If `substep_callback` is an int, we interpret it as a function pointer.
                If `substep_callback` is None, we disable substep_callbacks.
            userdata_names : list of strings or None
                This is a convenience parameter, if not None, this is passed
                onto ``model.set_userdata_names()``.
        '''
        if userdata_names is not None:
            self.model.set_userdata_names(userdata_names)
        if substep_callback is None:
            self.substep_callback_ptr = 0
        elif isinstance(substep_callback, int):
            self.substep_callback_ptr = substep_callback
        elif isinstance(substep_callback, str):
            self.substep_callback_ptr = build_callback_fn(substep_callback,
                                                          self.model.userdata_names)
        else:
            raise TypeError('invalid: {}'.format(type(substep_callback)))

    def step_udd(self):
        if self._udd_callback is None:
            self.udd_state = {}
        else:
            schema_example = self.udd_state
            self.udd_state = self._udd_callback(self)
            # Check to make sure the udd_state has consistent keys and dimension across steps
            if schema_example is not None:
                keys = set(schema_example.keys()) | set(self.udd_state.keys())
                for key in keys:
                    assert key in schema_example, "Keys cannot be added to udd_state between steps."
                    assert key in self.udd_state, "Keys cannot be dropped from udd_state between steps."
                    if isinstance(schema_example[key], Number):
                        assert isinstance(self.udd_state[key], Number), \
                            "Every value in udd_state must be either a number or a numpy array"
                    else:
                        assert isinstance(self.udd_state[key], np.ndarray), \
                            "Every value in udd_state must be either a number or a numpy array"
                        assert self.udd_state[key].shape == schema_example[key].shape, \
                            "Numpy array values in udd_state must keep the same dimension across steps."

    def get_state(self):
        """ Returns a copy of the simulator state. """
        qpos = np.copy(self.data.qpos)
        qvel = np.copy(self.data.qvel)
        if self.model.na == 0:
            act = None
        else:
            act = np.copy(self.data.act)
        udd_state = copy.deepcopy(self.udd_state)

        return MjSimState(self.data.time, qpos, qvel, act, udd_state)

    def set_state(self, value):
        """
        Sets the state from an MjSimState.
        If the MjSimState was previously unflattened from a numpy array, consider
        set_state_from_flattened, as the defensive copy is a substantial overhead
        in an inner loop.

        Args:
        - value (MjSimState): the desired state.
        - call_forward: optionally call sim.forward(). Called by default if
            the udd_callback is set.
        """
        self.data.time = value.time
        self.data.qpos[:] = np.copy(value.qpos)
        self.data.qvel[:] = np.copy(value.qvel)
        if self.model.na != 0:
            self.data.act[:] = np.copy(value.act)
        self.udd_state = copy.deepcopy(value.udd_state)

    def set_state_from_flattened(self, value):
        """ This helper method sets the state from an array without requiring a defensive copy."""
        state = MjSimState.from_flattened(value, self)

        self.data.time = state.time
        self.data.qpos[:] = state.qpos
        self.data.qvel[:] = state.qvel
        if self.model.na != 0:
            self.data.act[:] = state.act
        self.udd_state = state.udd_state

    def save(self, file, format='xml', keep_inertials=False):
        """
        Saves the simulator model and state to a file as either
        a MuJoCo XML or MJB file. The current state is saved as
        a keyframe in the model file. This is useful for debugging
        using MuJoCo's `simulate` utility.

        Note that this doesn't save the UDD-state which is
        part of MjSimState, since that's not supported natively
        by MuJoCo. If you want to save the model together with
        the UDD-state, you should use the `get_xml` or `get_mjb`
        methods on `MjModel` together with `MjSim.get_state` and
        save them with e.g. pickle.

        Args:
        - file (IO stream): stream to write model to.
        - format: format to use (either 'xml' or 'mjb')
        - keep_inertials (bool): if False, removes all <inertial>
          properties derived automatically for geoms by MuJoco. Note
          that this removes ones that were provided by the user
          as well.
        """
        xml_str = self.model.get_xml()
        dom = minidom.parseString(xml_str)

        mujoco_node = dom.childNodes[0]
        assert mujoco_node.tagName == 'mujoco'

        keyframe_el = dom.createElement('keyframe')
        key_el = dom.createElement('key')
        keyframe_el.appendChild(key_el)
        mujoco_node.appendChild(keyframe_el)

        def str_array(arr):
            return " ".join(map(str, arr))

        key_el.setAttribute('time', str(self.data.time))
        key_el.setAttribute('qpos', str_array(self.data.qpos))
        key_el.setAttribute('qvel', str_array(self.data.qvel))
        if self.data.act is not None:
            key_el.setAttribute('act', str_array(self.data.act))

        if not keep_inertials:
            for element in dom.getElementsByTagName('inertial'):
                element.parentNode.removeChild(element)

        result_xml = remove_empty_lines(dom.toprettyxml(indent=" " * 4))

        if format == 'xml':
            file.write(result_xml)
        elif format == 'mjb':
            new_model = load_model_from_xml(result_xml)
            file.write(new_model.get_mjb())
        else:
            raise ValueError("Unsupported format. Valid ones are 'xml' and 'mjb'")

    def ray(self, pnt, vec, include_static_geoms=True, exclude_body=-1, group_filter=None):
        """
        Cast a ray into the scene, and return the first valid geom it intersects.
            pnt - origin point of the ray in world coordinates (X Y Z)
            vec - direction of the ray in world coordinates (X Y Z)
            include_static_geoms - if False, we exclude geoms that are children of worldbody.
            exclude_body - if this is a body ID, we exclude all children geoms of this body.
            group_filter - a vector of booleans of length const.NGROUP
                           which specifies what geom groups (stored in model.geom_group)
                           to enable or disable.  If none, all groups are used
        Returns (distance, geomid) where
            distance - distance along ray until first collision with geom
            geomid - id of the geom the ray collided with
        If no collision was found in the scene, return (-1, None)

        NOTE: sometimes self.forward() needs to be called before self.ray().

        See self.ray_fast_group() and self.ray_fast_nogroup() for versions of this call
        with more stringent type requirements.
        """
        cdef mjtNum distance
        cdef mjtNum[::view.contiguous] pnt_view = pnt
        cdef mjtNum[::view.contiguous] vec_view = vec

        if group_filter is None:
            return self.ray_fast_nogroup(
                np.asarray(pnt, dtype=np.float64),
                np.asarray(vec, dtype=np.float64),
                1 if include_static_geoms else 0,
                exclude_body)
        else:
            return self.ray_fast_group(
                np.asarray(pnt, dtype=np.float64),
                np.asarray(vec, dtype=np.float64),
                np.asarray(group_filter, dtype=np.uint8),
                1 if include_static_geoms else 0,
                exclude_body)

    def ray_fast_group(self,
            np.ndarray[np.float64_t, mode="c", ndim=1] pnt,
            np.ndarray[np.float64_t, mode="c", ndim=1] vec,
            np.ndarray[np.uint8_t, mode="c", ndim=1] geomgroup,
            mjtByte flg_static=1,
            int bodyexclude=-1):
        """
        Faster version of sim.ray(), which avoids extra copies,
        but needs to be given all the correct type arrays.

        See self.ray() for explanation of arguments
        """
        cdef int geomid
        cdef mjtNum distance
        cdef mjtNum[::view.contiguous] pnt_view = pnt
        cdef mjtNum[::view.contiguous] vec_view = vec
        cdef mjtByte[::view.contiguous] geomgroup_view = geomgroup

        distance = mj_ray(self.model.ptr,
                          self.data.ptr,
                          &pnt_view[0],
                          &vec_view[0],
                          &geomgroup_view[0],
                          flg_static,
                          bodyexclude,
                          &geomid)
        return (distance, geomid)


    def ray_fast_nogroup(self,
            np.ndarray[np.float64_t, mode="c", ndim=1] pnt,
            np.ndarray[np.float64_t, mode="c", ndim=1] vec,
            mjtByte flg_static=1,
            int bodyexclude=-1):
        """
        Faster version of sim.ray(), which avoids extra copies,
        but needs to be given all the correct type arrays.

        This version hardcodes the geomgroup to NULL.
        (Can't easily express a signature that is "numpy array of specific type or None")

        See self.ray() for explanation of arguments
        """
        cdef int geomid
        cdef mjtNum distance
        cdef mjtNum[::view.contiguous] pnt_view = pnt
        cdef mjtNum[::view.contiguous] vec_view = vec

        distance = mj_ray(self.model.ptr,
                          self.data.ptr,
                          &pnt_view[0],
                          &vec_view[0],
                          NULL,
                          flg_static,
                          bodyexclude,
                          &geomid)
        return (distance, geomid)
