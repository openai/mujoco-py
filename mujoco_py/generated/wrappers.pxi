# cython: language_level=3
# Automatically generated. Do not modify!

include "../pxd/mujoco.pxd"
from cpython.mem cimport PyMem_Malloc, PyMem_Free
cimport numpy as np
import numpy as np
from tempfile import TemporaryDirectory

cdef class PyMjvFigure(object):
    cdef mjvFigure* ptr
    
    
    cdef np.ndarray _flg_ticklabel
    cdef np.ndarray _gridsize
    cdef np.ndarray _gridrgb
    cdef np.ndarray _figurergba
    cdef np.ndarray _legendrgba
    cdef np.ndarray _textrgb
    cdef np.ndarray _xlabel
    cdef np.ndarray _title
    cdef np.ndarray _xformat
    cdef np.ndarray _yformat
    cdef np.ndarray _minwidth
    cdef np.ndarray _linepnt
    cdef np.ndarray _linewidth
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjvFigure* p):
        
        self.ptr = p
        
        
        self._flg_ticklabel = _wrap_int_1d(&p.flg_ticklabel[0], 2)
        self._gridsize = _wrap_int_1d(&p.gridsize[0], 2)
        self._gridrgb = _wrap_float_1d(&p.gridrgb[0], 3)
        self._figurergba = _wrap_float_1d(&p.figurergba[0], 4)
        self._legendrgba = _wrap_float_1d(&p.legendrgba[0], 4)
        self._textrgb = _wrap_float_1d(&p.textrgb[0], 3)
        self._xlabel = _wrap_char_1d(&p.xlabel[0], 100)
        self._title = _wrap_char_1d(&p.title[0], 100)
        self._xformat = _wrap_char_1d(&p.xformat[0], 20)
        self._yformat = _wrap_char_1d(&p.yformat[0], 20)
        self._minwidth = _wrap_char_1d(&p.minwidth[0], 20)
        self._linepnt = _wrap_int_1d(&p.linepnt[0], 100)
        self._linewidth = _wrap_float_1d(&p.linewidth[0], 100)
        
    @property
    def flg_legend(self): return self.ptr.flg_legend
    @flg_legend.setter
    def flg_legend(self, int x): self.ptr.flg_legend = x
    @property
    def flg_extend(self): return self.ptr.flg_extend
    @flg_extend.setter
    def flg_extend(self, int x): self.ptr.flg_extend = x
    @property
    def flg_barplot(self): return self.ptr.flg_barplot
    @flg_barplot.setter
    def flg_barplot(self, int x): self.ptr.flg_barplot = x
    @property
    def gridwidth(self): return self.ptr.gridwidth
    @gridwidth.setter
    def gridwidth(self, float x): self.ptr.gridwidth = x
    @property
    def flg_ticklabel(self): return self._flg_ticklabel
    @property
    def gridsize(self): return self._gridsize
    @property
    def gridrgb(self): return self._gridrgb
    @property
    def figurergba(self): return self._figurergba
    @property
    def legendrgba(self): return self._legendrgba
    @property
    def textrgb(self): return self._textrgb
    @property
    def xlabel(self): return self._xlabel
    @property
    def title(self): return self._title
    @property
    def xformat(self): return self._xformat
    @property
    def yformat(self): return self._yformat
    @property
    def minwidth(self): return self._minwidth
    @property
    def linepnt(self): return self._linepnt
    @property
    def linewidth(self): return self._linewidth

cdef PyMjvFigure WrapMjvFigure(mjvFigure* p):
    cdef PyMjvFigure o = PyMjvFigure()
    o._set(p)
    return o

cdef class PyMjvGLCamera(object):
    cdef mjvGLCamera* ptr
    
    
    cdef np.ndarray _pos
    cdef np.ndarray _forward
    cdef np.ndarray _up
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjvGLCamera* p):
        
        self.ptr = p
        
        
        self._pos = _wrap_float_1d(&p.pos[0], 3)
        self._forward = _wrap_float_1d(&p.forward[0], 3)
        self._up = _wrap_float_1d(&p.up[0], 3)
        
    @property
    def frustum_center(self): return self.ptr.frustum_center
    @frustum_center.setter
    def frustum_center(self, float x): self.ptr.frustum_center = x
    @property
    def frustum_bottom(self): return self.ptr.frustum_bottom
    @frustum_bottom.setter
    def frustum_bottom(self, float x): self.ptr.frustum_bottom = x
    @property
    def frustum_top(self): return self.ptr.frustum_top
    @frustum_top.setter
    def frustum_top(self, float x): self.ptr.frustum_top = x
    @property
    def frustum_near(self): return self.ptr.frustum_near
    @frustum_near.setter
    def frustum_near(self, float x): self.ptr.frustum_near = x
    @property
    def frustum_far(self): return self.ptr.frustum_far
    @frustum_far.setter
    def frustum_far(self, float x): self.ptr.frustum_far = x
    @property
    def pos(self): return self._pos
    @property
    def forward(self): return self._forward
    @property
    def up(self): return self._up

cdef PyMjvGLCamera WrapMjvGLCamera(mjvGLCamera* p):
    cdef PyMjvGLCamera o = PyMjvGLCamera()
    o._set(p)
    return o

cdef class PyMjvPerturb(object):
    cdef mjvPerturb* ptr
    
    
    cdef np.ndarray _refpos
    cdef np.ndarray _refquat
    cdef np.ndarray _localpos
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjvPerturb* p):
        
        self.ptr = p
        
        
        self._refpos = _wrap_mjtNum_1d(&p.refpos[0], 3)
        self._refquat = _wrap_mjtNum_1d(&p.refquat[0], 4)
        self._localpos = _wrap_mjtNum_1d(&p.localpos[0], 3)
        
    @property
    def select(self): return self.ptr.select
    @select.setter
    def select(self, int x): self.ptr.select = x
    @property
    def active(self): return self.ptr.active
    @active.setter
    def active(self, int x): self.ptr.active = x
    @property
    def scale(self): return self.ptr.scale
    @scale.setter
    def scale(self, mjtNum x): self.ptr.scale = x
    @property
    def refpos(self): return self._refpos
    @property
    def refquat(self): return self._refquat
    @property
    def localpos(self): return self._localpos

cdef PyMjvPerturb WrapMjvPerturb(mjvPerturb* p):
    cdef PyMjvPerturb o = PyMjvPerturb()
    o._set(p)
    return o

cdef class PyMjSolverStat(object):
    cdef mjSolverStat* ptr
    
    
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjSolverStat* p):
        
        self.ptr = p
        
        
        
    @property
    def improvement(self): return self.ptr.improvement
    @improvement.setter
    def improvement(self, mjtNum x): self.ptr.improvement = x
    @property
    def gradient(self): return self.ptr.gradient
    @gradient.setter
    def gradient(self, mjtNum x): self.ptr.gradient = x
    @property
    def lineslope(self): return self.ptr.lineslope
    @lineslope.setter
    def lineslope(self, mjtNum x): self.ptr.lineslope = x
    @property
    def nactive(self): return self.ptr.nactive
    @nactive.setter
    def nactive(self, int x): self.ptr.nactive = x
    @property
    def nchange(self): return self.ptr.nchange
    @nchange.setter
    def nchange(self, int x): self.ptr.nchange = x
    @property
    def neval(self): return self.ptr.neval
    @neval.setter
    def neval(self, int x): self.ptr.neval = x
    @property
    def nupdate(self): return self.ptr.nupdate
    @nupdate.setter
    def nupdate(self, int x): self.ptr.nupdate = x

cdef PyMjSolverStat WrapMjSolverStat(mjSolverStat* p):
    cdef PyMjSolverStat o = PyMjSolverStat()
    o._set(p)
    return o

cdef class PyMjWarningStat(object):
    cdef mjWarningStat* ptr
    
    
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjWarningStat* p):
        
        self.ptr = p
        
        
        
    @property
    def lastinfo(self): return self.ptr.lastinfo
    @lastinfo.setter
    def lastinfo(self, int x): self.ptr.lastinfo = x
    @property
    def number(self): return self.ptr.number
    @number.setter
    def number(self, int x): self.ptr.number = x

cdef PyMjWarningStat WrapMjWarningStat(mjWarningStat* p):
    cdef PyMjWarningStat o = PyMjWarningStat()
    o._set(p)
    return o

cdef class PyMjVisual(object):
    cdef mjVisual* ptr
    
    
    cdef PyMjVisual_global_ _global_
    cdef PyMjVisual_quality _quality
    cdef PyMjVisual_headlight _headlight
    cdef PyMjVisual_map _map
    cdef PyMjVisual_scale _scale
    cdef PyMjVisual_rgba _rgba
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjVisual* p):
        
        self.ptr = p
        
        
        self._global_ = WrapMjVisual_global_(&p.global_)
        self._quality = WrapMjVisual_quality(&p.quality)
        self._headlight = WrapMjVisual_headlight(&p.headlight)
        self._map = WrapMjVisual_map(&p.map)
        self._scale = WrapMjVisual_scale(&p.scale)
        self._rgba = WrapMjVisual_rgba(&p.rgba)
        
    @property
    def global_(self): return self._global_
    @property
    def quality(self): return self._quality
    @property
    def headlight(self): return self._headlight
    @property
    def map(self): return self._map
    @property
    def scale(self): return self._scale
    @property
    def rgba(self): return self._rgba

cdef PyMjVisual WrapMjVisual(mjVisual* p):
    cdef PyMjVisual o = PyMjVisual()
    o._set(p)
    return o

cdef class PyMjVisual_quality(object):
    cdef mjVisual_quality* ptr
    
    
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjVisual_quality* p):
        
        self.ptr = p
        
        
        
    @property
    def shadowsize(self): return self.ptr.shadowsize
    @shadowsize.setter
    def shadowsize(self, int x): self.ptr.shadowsize = x
    @property
    def offsamples(self): return self.ptr.offsamples
    @offsamples.setter
    def offsamples(self, int x): self.ptr.offsamples = x
    @property
    def numslices(self): return self.ptr.numslices
    @numslices.setter
    def numslices(self, int x): self.ptr.numslices = x
    @property
    def numstacks(self): return self.ptr.numstacks
    @numstacks.setter
    def numstacks(self, int x): self.ptr.numstacks = x
    @property
    def numarrows(self): return self.ptr.numarrows
    @numarrows.setter
    def numarrows(self, int x): self.ptr.numarrows = x
    @property
    def numquads(self): return self.ptr.numquads
    @numquads.setter
    def numquads(self, int x): self.ptr.numquads = x

cdef PyMjVisual_quality WrapMjVisual_quality(mjVisual_quality* p):
    cdef PyMjVisual_quality o = PyMjVisual_quality()
    o._set(p)
    return o

cdef class PyMjOption(object):
    cdef mjOption* ptr
    
    
    cdef np.ndarray _gravity
    cdef np.ndarray _wind
    cdef np.ndarray _magnetic
    cdef np.ndarray _o_solref
    cdef np.ndarray _o_solimp
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjOption* p):
        
        self.ptr = p
        
        
        self._gravity = _wrap_mjtNum_1d(&p.gravity[0], 3)
        self._wind = _wrap_mjtNum_1d(&p.wind[0], 3)
        self._magnetic = _wrap_mjtNum_1d(&p.magnetic[0], 3)
        self._o_solref = _wrap_mjtNum_1d(&p.o_solref[0], 2)
        self._o_solimp = _wrap_mjtNum_1d(&p.o_solimp[0], 3)
        
    @property
    def timestep(self): return self.ptr.timestep
    @timestep.setter
    def timestep(self, mjtNum x): self.ptr.timestep = x
    @property
    def apirate(self): return self.ptr.apirate
    @apirate.setter
    def apirate(self, mjtNum x): self.ptr.apirate = x
    @property
    def impratio(self): return self.ptr.impratio
    @impratio.setter
    def impratio(self, mjtNum x): self.ptr.impratio = x
    @property
    def tolerance(self): return self.ptr.tolerance
    @tolerance.setter
    def tolerance(self, mjtNum x): self.ptr.tolerance = x
    @property
    def noslip_tolerance(self): return self.ptr.noslip_tolerance
    @noslip_tolerance.setter
    def noslip_tolerance(self, mjtNum x): self.ptr.noslip_tolerance = x
    @property
    def mpr_tolerance(self): return self.ptr.mpr_tolerance
    @mpr_tolerance.setter
    def mpr_tolerance(self, mjtNum x): self.ptr.mpr_tolerance = x
    @property
    def density(self): return self.ptr.density
    @density.setter
    def density(self, mjtNum x): self.ptr.density = x
    @property
    def viscosity(self): return self.ptr.viscosity
    @viscosity.setter
    def viscosity(self, mjtNum x): self.ptr.viscosity = x
    @property
    def o_margin(self): return self.ptr.o_margin
    @o_margin.setter
    def o_margin(self, mjtNum x): self.ptr.o_margin = x
    @property
    def integrator(self): return self.ptr.integrator
    @integrator.setter
    def integrator(self, int x): self.ptr.integrator = x
    @property
    def collision(self): return self.ptr.collision
    @collision.setter
    def collision(self, int x): self.ptr.collision = x
    @property
    def impedance(self): return self.ptr.impedance
    @impedance.setter
    def impedance(self, int x): self.ptr.impedance = x
    @property
    def reference(self): return self.ptr.reference
    @reference.setter
    def reference(self, int x): self.ptr.reference = x
    @property
    def cone(self): return self.ptr.cone
    @cone.setter
    def cone(self, int x): self.ptr.cone = x
    @property
    def jacobian(self): return self.ptr.jacobian
    @jacobian.setter
    def jacobian(self, int x): self.ptr.jacobian = x
    @property
    def solver(self): return self.ptr.solver
    @solver.setter
    def solver(self, int x): self.ptr.solver = x
    @property
    def iterations(self): return self.ptr.iterations
    @iterations.setter
    def iterations(self, int x): self.ptr.iterations = x
    @property
    def noslip_iterations(self): return self.ptr.noslip_iterations
    @noslip_iterations.setter
    def noslip_iterations(self, int x): self.ptr.noslip_iterations = x
    @property
    def mpr_iterations(self): return self.ptr.mpr_iterations
    @mpr_iterations.setter
    def mpr_iterations(self, int x): self.ptr.mpr_iterations = x
    @property
    def disableflags(self): return self.ptr.disableflags
    @disableflags.setter
    def disableflags(self, int x): self.ptr.disableflags = x
    @property
    def enableflags(self): return self.ptr.enableflags
    @enableflags.setter
    def enableflags(self, int x): self.ptr.enableflags = x
    @property
    def gravity(self): return self._gravity
    @property
    def wind(self): return self._wind
    @property
    def magnetic(self): return self._magnetic
    @property
    def o_solref(self): return self._o_solref
    @property
    def o_solimp(self): return self._o_solimp

cdef PyMjOption WrapMjOption(mjOption* p):
    cdef PyMjOption o = PyMjOption()
    o._set(p)
    return o

cdef class PyMjvOption(object):
    cdef mjvOption* ptr
    
    
    cdef np.ndarray _geomgroup
    cdef np.ndarray _sitegroup
    cdef np.ndarray _flags
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjvOption* p):
        
        self.ptr = p
        
        
        self._geomgroup = _wrap_mjtByte_1d(&p.geomgroup[0], 5)
        self._sitegroup = _wrap_mjtByte_1d(&p.sitegroup[0], 5)
        self._flags = _wrap_mjtByte_1d(&p.flags[0], mjNVISFLAG)
        
    @property
    def label(self): return self.ptr.label
    @label.setter
    def label(self, int x): self.ptr.label = x
    @property
    def frame(self): return self.ptr.frame
    @frame.setter
    def frame(self, int x): self.ptr.frame = x
    @property
    def geomgroup(self): return self._geomgroup
    @property
    def sitegroup(self): return self._sitegroup
    @property
    def flags(self): return self._flags

cdef PyMjvOption WrapMjvOption(mjvOption* p):
    cdef PyMjvOption o = PyMjvOption()
    o._set(p)
    return o

cdef class PyMjVisual_headlight(object):
    cdef mjVisual_headlight* ptr
    
    
    cdef np.ndarray _ambient
    cdef np.ndarray _diffuse
    cdef np.ndarray _specular
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjVisual_headlight* p):
        
        self.ptr = p
        
        
        self._ambient = _wrap_float_1d(&p.ambient[0], 3)
        self._diffuse = _wrap_float_1d(&p.diffuse[0], 3)
        self._specular = _wrap_float_1d(&p.specular[0], 3)
        
    @property
    def active(self): return self.ptr.active
    @active.setter
    def active(self, int x): self.ptr.active = x
    @property
    def ambient(self): return self._ambient
    @property
    def diffuse(self): return self._diffuse
    @property
    def specular(self): return self._specular

cdef PyMjVisual_headlight WrapMjVisual_headlight(mjVisual_headlight* p):
    cdef PyMjVisual_headlight o = PyMjVisual_headlight()
    o._set(p)
    return o

cdef class PyMjVisual_map(object):
    cdef mjVisual_map* ptr
    
    
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjVisual_map* p):
        
        self.ptr = p
        
        
        
    @property
    def stiffness(self): return self.ptr.stiffness
    @stiffness.setter
    def stiffness(self, float x): self.ptr.stiffness = x
    @property
    def stiffnessrot(self): return self.ptr.stiffnessrot
    @stiffnessrot.setter
    def stiffnessrot(self, float x): self.ptr.stiffnessrot = x
    @property
    def force(self): return self.ptr.force
    @force.setter
    def force(self, float x): self.ptr.force = x
    @property
    def torque(self): return self.ptr.torque
    @torque.setter
    def torque(self, float x): self.ptr.torque = x
    @property
    def alpha(self): return self.ptr.alpha
    @alpha.setter
    def alpha(self, float x): self.ptr.alpha = x
    @property
    def fogstart(self): return self.ptr.fogstart
    @fogstart.setter
    def fogstart(self, float x): self.ptr.fogstart = x
    @property
    def fogend(self): return self.ptr.fogend
    @fogend.setter
    def fogend(self, float x): self.ptr.fogend = x
    @property
    def znear(self): return self.ptr.znear
    @znear.setter
    def znear(self, float x): self.ptr.znear = x
    @property
    def zfar(self): return self.ptr.zfar
    @zfar.setter
    def zfar(self, float x): self.ptr.zfar = x
    @property
    def shadowclip(self): return self.ptr.shadowclip
    @shadowclip.setter
    def shadowclip(self, float x): self.ptr.shadowclip = x
    @property
    def shadowscale(self): return self.ptr.shadowscale
    @shadowscale.setter
    def shadowscale(self, float x): self.ptr.shadowscale = x

cdef PyMjVisual_map WrapMjVisual_map(mjVisual_map* p):
    cdef PyMjVisual_map o = PyMjVisual_map()
    o._set(p)
    return o

cdef class PyMjrRect(object):
    cdef mjrRect* ptr
    
    
    
    def __cinit__(self):
        self.ptr = <mjrRect*> PyMem_Malloc(sizeof(mjrRect))
        if not self.ptr:
            raise MemoryError()

    def __dealloc__(self):
        PyMem_Free(self.ptr)

    cdef void _set(self, mjrRect* p):
        
        self.ptr = p
        
        
        
    @property
    def left(self): return self.ptr.left
    @left.setter
    def left(self, int x): self.ptr.left = x
    @property
    def bottom(self): return self.ptr.bottom
    @bottom.setter
    def bottom(self, int x): self.ptr.bottom = x
    @property
    def width(self): return self.ptr.width
    @width.setter
    def width(self, int x): self.ptr.width = x
    @property
    def height(self): return self.ptr.height
    @height.setter
    def height(self, int x): self.ptr.height = x

cdef PyMjrRect WrapMjrRect(mjrRect* p):
    cdef PyMjrRect o = PyMjrRect()
    o._set(p)
    return o

cdef class PyMjvCamera(object):
    cdef mjvCamera* ptr
    
    
    cdef np.ndarray _lookat
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjvCamera* p):
        
        self.ptr = p
        
        
        self._lookat = _wrap_mjtNum_1d(&p.lookat[0], 3)
        
    @property
    def type(self): return self.ptr.type
    @type.setter
    def type(self, int x): self.ptr.type = x
    @property
    def fixedcamid(self): return self.ptr.fixedcamid
    @fixedcamid.setter
    def fixedcamid(self, int x): self.ptr.fixedcamid = x
    @property
    def trackbodyid(self): return self.ptr.trackbodyid
    @trackbodyid.setter
    def trackbodyid(self, int x): self.ptr.trackbodyid = x
    @property
    def distance(self): return self.ptr.distance
    @distance.setter
    def distance(self, mjtNum x): self.ptr.distance = x
    @property
    def azimuth(self): return self.ptr.azimuth
    @azimuth.setter
    def azimuth(self, mjtNum x): self.ptr.azimuth = x
    @property
    def elevation(self): return self.ptr.elevation
    @elevation.setter
    def elevation(self, mjtNum x): self.ptr.elevation = x
    @property
    def lookat(self): return self._lookat

cdef PyMjvCamera WrapMjvCamera(mjvCamera* p):
    cdef PyMjvCamera o = PyMjvCamera()
    o._set(p)
    return o

cdef class PyMjData(object):
    cdef mjData* ptr
    
    cdef PyMjModel _model
    
    cdef np.ndarray _qpos
    cdef np.ndarray _qvel
    cdef np.ndarray _act
    cdef np.ndarray _ctrl
    cdef np.ndarray _qfrc_applied
    cdef np.ndarray _xfrc_applied
    cdef np.ndarray _qacc
    cdef np.ndarray _act_dot
    cdef np.ndarray _mocap_pos
    cdef np.ndarray _mocap_quat
    cdef np.ndarray _userdata
    cdef np.ndarray _sensordata
    cdef np.ndarray _xpos
    cdef np.ndarray _xquat
    cdef np.ndarray _xmat
    cdef np.ndarray _xipos
    cdef np.ndarray _ximat
    cdef np.ndarray _xanchor
    cdef np.ndarray _xaxis
    cdef np.ndarray _geom_xpos
    cdef np.ndarray _geom_xmat
    cdef np.ndarray _site_xpos
    cdef np.ndarray _site_xmat
    cdef np.ndarray _cam_xpos
    cdef np.ndarray _cam_xmat
    cdef np.ndarray _light_xpos
    cdef np.ndarray _light_xdir
    cdef np.ndarray _subtree_com
    cdef np.ndarray _cdof
    cdef np.ndarray _cinert
    cdef np.ndarray _ten_wrapadr
    cdef np.ndarray _ten_wrapnum
    cdef np.ndarray _ten_length
    cdef np.ndarray _ten_moment
    cdef np.ndarray _wrap_obj
    cdef np.ndarray _wrap_xpos
    cdef np.ndarray _actuator_length
    cdef np.ndarray _actuator_moment
    cdef np.ndarray _crb
    cdef np.ndarray _qM
    cdef np.ndarray _qLD
    cdef np.ndarray _qLDiagInv
    cdef np.ndarray _qLDiagSqrtInv
    cdef tuple _contact
    cdef np.ndarray _efc_type
    cdef np.ndarray _efc_id
    cdef np.ndarray _efc_J_rownnz
    cdef np.ndarray _efc_J_rowadr
    cdef np.ndarray _efc_J_colind
    cdef np.ndarray _efc_JT_rownnz
    cdef np.ndarray _efc_JT_rowadr
    cdef np.ndarray _efc_JT_colind
    cdef np.ndarray _efc_solref
    cdef np.ndarray _efc_solimp
    cdef np.ndarray _efc_margin
    cdef np.ndarray _efc_frictionloss
    cdef np.ndarray _efc_pos
    cdef np.ndarray _efc_J
    cdef np.ndarray _efc_JT
    cdef np.ndarray _efc_diagApprox
    cdef np.ndarray _efc_D
    cdef np.ndarray _efc_R
    cdef np.ndarray _efc_AR_rownnz
    cdef np.ndarray _efc_AR_rowadr
    cdef np.ndarray _efc_AR_colind
    cdef np.ndarray _efc_AR
    cdef np.ndarray _ten_velocity
    cdef np.ndarray _actuator_velocity
    cdef np.ndarray _cvel
    cdef np.ndarray _cdof_dot
    cdef np.ndarray _qfrc_bias
    cdef np.ndarray _qfrc_passive
    cdef np.ndarray _efc_vel
    cdef np.ndarray _efc_aref
    cdef np.ndarray _subtree_linvel
    cdef np.ndarray _subtree_angmom
    cdef np.ndarray _actuator_force
    cdef np.ndarray _qfrc_actuator
    cdef np.ndarray _qfrc_unc
    cdef np.ndarray _qacc_unc
    cdef np.ndarray _efc_b
    cdef np.ndarray _efc_force
    cdef np.ndarray _efc_state
    cdef np.ndarray _qfrc_constraint
    cdef np.ndarray _qacc_warmstart
    cdef np.ndarray _qfrc_inverse
    cdef np.ndarray _cacc
    cdef np.ndarray _cfrc_int
    cdef np.ndarray _cfrc_ext
    cdef list _warning
    cdef list _timer
    cdef list _solver
    cdef np.ndarray _solver_fwdinv
    cdef np.ndarray _energy
    
    @property
    def body_xpos(self):
        return self._xpos

    @property
    def body_xquat(self):
        return self._xquat

    @property
    def body_xmat(self):
        return self._xmat

    @property
    def active_contacts_efc_pos(self):
        return self._efc_pos[self.ne:self.nefc]

    def __dealloc__(self):
        mj_deleteData(self.ptr)


    def get_body_xpos(self, name):
        id = self._model.body_name2id(name)
        return self._xpos[id]

    def get_xpos(self, name):
        raise RuntimeError("get_body_xpos should be used instead of get_xpos")

    def get_body_xquat(self, name):
        id = self._model.body_name2id(name)
        return self._xquat[id]

    def get_xquat(self, name):
        raise RuntimeError("get_body_xquat should be used instead of get_xquat")

    def get_body_xmat(self, name):
        id = self._model.body_name2id(name)
        return self._xmat[id].reshape((3, 3))

    def get_xmat(self, name):
        raise RuntimeError("get_body_xmat should be used instead of get_xmat")

    def get_body_xipos(self, name):
        id = self._model.body_name2id(name)
        return self._xipos[id]

    def get_xipos(self, name):
        raise RuntimeError("get_body_xipos should be used instead of get_xipos")

    def get_body_ximat(self, name):
        id = self._model.body_name2id(name)
        return self._ximat[id].reshape((3, 3))

    def get_ximat(self, name):
        raise RuntimeError("get_body_ximat should be used instead of get_ximat")

    def get_body_jacp(self, name, np.ndarray[double, ndim=1, mode="c"] jacp = None):
        id = self._model.body_name2id(name)
        if jacp is None:
            jacp = np.zeros(3 * self._model.nv)
        cdef double * jacp_view = &jacp[0]
        mj_jacBody(self._model.ptr, self.ptr, jacp_view, NULL, id)
        return jacp

    def get_body_jacr(self, name, np.ndarray[double, ndim=1, mode="c"] jacr = None):
        id = self._model.body_name2id(name)
        if jacr is None:
            jacr = np.zeros(3 * self._model.nv)
        cdef double * jacr_view = &jacr[0]
        mj_jacBody(self._model.ptr, self.ptr, NULL, jacr_view, id)
        return jacr

    def get_body_xvelp(self, name):
        id = self._model.body_name2id(name)
        jacp = self.get_body_jacp(name).reshape((3, self._model.nv))
        xvelp = np.dot(jacp, self.qvel)
        return xvelp

    def get_body_xvelr(self, name):
        id = self._model.body_name2id(name)
        jacr = self.get_body_jacr(name).reshape((3, self._model.nv))
        xvelr = np.dot(jacr, self.qvel)
        return xvelr

    def get_joint_xanchor(self, name):
        id = self._model.joint_name2id(name)
        return self._xanchor[id]

    def get_xanchor(self, name):
        raise RuntimeError("get_joint_xanchor should be used instead of get_xanchor")

    def get_joint_xaxis(self, name):
        id = self._model.joint_name2id(name)
        return self._xaxis[id]

    def get_xaxis(self, name):
        raise RuntimeError("get_joint_xaxis should be used instead of get_xaxis")

    def get_geom_xpos(self, name):
        id = self._model.geom_name2id(name)
        return self._geom_xpos[id]

    def get_geom_xmat(self, name):
        id = self._model.geom_name2id(name)
        return self._geom_xmat[id].reshape((3, 3))

    def get_geom_jacp(self, name, np.ndarray[double, ndim=1, mode="c"] jacp = None):
        id = self._model.geom_name2id(name)
        if jacp is None:
            jacp = np.zeros(3 * self._model.nv)
        cdef double * jacp_view = &jacp[0]
        mj_jacGeom(self._model.ptr, self.ptr, jacp_view, NULL, id)
        return jacp

    def get_geom_jacr(self, name, np.ndarray[double, ndim=1, mode="c"] jacr = None):
        id = self._model.geom_name2id(name)
        if jacr is None:
            jacr = np.zeros(3 * self._model.nv)
        cdef double * jacr_view = &jacr[0]
        mj_jacGeom(self._model.ptr, self.ptr, NULL, jacr_view, id)
        return jacr

    def get_geom_xvelp(self, name):
        id = self._model.geom_name2id(name)
        jacp = self.get_geom_jacp(name).reshape((3, self._model.nv))
        xvelp = np.dot(jacp, self.qvel)
        return xvelp

    def get_geom_xvelr(self, name):
        id = self._model.geom_name2id(name)
        jacr = self.get_geom_jacr(name).reshape((3, self._model.nv))
        xvelr = np.dot(jacr, self.qvel)
        return xvelr

    def get_site_xpos(self, name):
        id = self._model.site_name2id(name)
        return self._site_xpos[id]

    def get_site_xmat(self, name):
        id = self._model.site_name2id(name)
        return self._site_xmat[id].reshape((3, 3))

    def get_site_jacp(self, name, np.ndarray[double, ndim=1, mode="c"] jacp = None):
        id = self._model.site_name2id(name)
        if jacp is None:
            jacp = np.zeros(3 * self._model.nv)
        cdef double * jacp_view = &jacp[0]
        mj_jacSite(self._model.ptr, self.ptr, jacp_view, NULL, id)
        return jacp

    def get_site_jacr(self, name, np.ndarray[double, ndim=1, mode="c"] jacr = None):
        id = self._model.site_name2id(name)
        if jacr is None:
            jacr = np.zeros(3 * self._model.nv)
        cdef double * jacr_view = &jacr[0]
        mj_jacSite(self._model.ptr, self.ptr, NULL, jacr_view, id)
        return jacr

    def get_site_xvelp(self, name):
        id = self._model.site_name2id(name)
        jacp = self.get_site_jacp(name).reshape((3, self._model.nv))
        xvelp = np.dot(jacp, self.qvel)
        return xvelp

    def get_site_xvelr(self, name):
        id = self._model.site_name2id(name)
        jacr = self.get_site_jacr(name).reshape((3, self._model.nv))
        xvelr = np.dot(jacr, self.qvel)
        return xvelr

    def get_camera_xpos(self, name):
        id = self._model.camera_name2id(name)
        return self._cam_xpos[id]

    def get_cam_xpos(self, name):
        raise RuntimeError("get_camera_xpos should be used instead of get_cam_xpos")

    def get_camera_xmat(self, name):
        id = self._model.camera_name2id(name)
        return self._cam_xmat[id].reshape((3, 3))

    def get_cam_xmat(self, name):
        raise RuntimeError("get_camera_xmat should be used instead of get_cam_xmat")

    def get_light_xpos(self, name):
        id = self._model.light_name2id(name)
        return self._light_xpos[id]

    def get_light_xdir(self, name):
        id = self._model.light_name2id(name)
        return self._light_xdir[id]

    def get_sensor(self, name):
        id = self._model.sensor_name2id(name)
        return self._sensordata[id]

    def get_sensordata(self, name):
        raise RuntimeError("get_sensor should be used instead of get_sensordata")

    def get_mocap_pos(self, name):
        body_id = self._model.body_name2id(name)
        mocap_id = self._model.body_mocapid[body_id]
        return self.mocap_pos[mocap_id]

    def set_mocap_pos(self, name, value):
        body_id = self._model.body_name2id(name)
        mocap_id = self._model.body_mocapid[body_id]
        self.mocap_pos[mocap_id] = value

    def get_mocap_quat(self, name):
        body_id = self._model.body_name2id(name)
        mocap_id = self._model.body_mocapid[body_id]
        return self.mocap_quat[mocap_id]

    def set_mocap_quat(self, name, value):
        body_id = self._model.body_name2id(name)
        mocap_id = self._model.body_mocapid[body_id]
        self.mocap_quat[mocap_id] = value

    def get_joint_qpos(self, name):
        addr = self._model.get_joint_qpos_addr(name)
        if isinstance(addr, (int, np.int32, np.int64)):
            return self.qpos[addr]
        else:
            start_i, end_i = addr
            return self.qpos[start_i:end_i]

    def set_joint_qpos(self, name, value):
        addr = self._model.get_joint_qpos_addr(name)
        if isinstance(addr, (int, np.int32, np.int64)):
            self.qpos[addr] = value
        else:
            start_i, end_i = addr
            value = np.array(value)
            assert value.shape == (end_i - start_i,), (
                "Value has incorrect shape %s: %s" % (name, value))
            self.qpos[start_i:end_i] = value

    def get_joint_qvel(self, name):
        addr = self._model.get_joint_qvel_addr(name)
        if isinstance(addr, (int, np.int32, np.int64)):
            return self.qvel[addr]
        else:
            start_i, end_i = addr
            return self.qvel[start_i:end_i]

    def set_joint_qvel(self, name, value):
        addr = self._model.get_joint_qvel_addr(name)
        if isinstance(addr, (int, np.int32, np.int64)):
            self.qvel[addr] = value
        else:
            start_i, end_i = addr
            value = np.array(value)
            assert value.shape == (end_i - start_i,), (
                "Value has incorrect shape %s: %s" % (name, value))
            self.qvel[start_i:end_i] = value

    cdef void _set(self, mjData* p, PyMjModel model):
        
        self.ptr = p
        self._model = model
        
        self._qpos = _wrap_mjtNum_1d(p.qpos, model.nq)
        self._qvel = _wrap_mjtNum_1d(p.qvel, model.nv)
        self._act = _wrap_mjtNum_1d(p.act, model.na)
        self._ctrl = _wrap_mjtNum_1d(p.ctrl, model.nu)
        self._qfrc_applied = _wrap_mjtNum_1d(p.qfrc_applied, model.nv)
        self._xfrc_applied = _wrap_mjtNum_2d(p.xfrc_applied, model.nbody, 6)
        self._qacc = _wrap_mjtNum_1d(p.qacc, model.nv)
        self._act_dot = _wrap_mjtNum_1d(p.act_dot, model.na)
        self._mocap_pos = _wrap_mjtNum_2d(p.mocap_pos, model.nmocap, 3)
        self._mocap_quat = _wrap_mjtNum_2d(p.mocap_quat, model.nmocap, 4)
        self._userdata = _wrap_mjtNum_1d(p.userdata, model.nuserdata)
        self._sensordata = _wrap_mjtNum_1d(p.sensordata, model.nsensordata)
        self._xpos = _wrap_mjtNum_2d(p.xpos, model.nbody, 3)
        self._xquat = _wrap_mjtNum_2d(p.xquat, model.nbody, 4)
        self._xmat = _wrap_mjtNum_2d(p.xmat, model.nbody, 9)
        self._xipos = _wrap_mjtNum_2d(p.xipos, model.nbody, 3)
        self._ximat = _wrap_mjtNum_2d(p.ximat, model.nbody, 9)
        self._xanchor = _wrap_mjtNum_2d(p.xanchor, model.njnt, 3)
        self._xaxis = _wrap_mjtNum_2d(p.xaxis, model.njnt, 3)
        self._geom_xpos = _wrap_mjtNum_2d(p.geom_xpos, model.ngeom, 3)
        self._geom_xmat = _wrap_mjtNum_2d(p.geom_xmat, model.ngeom, 9)
        self._site_xpos = _wrap_mjtNum_2d(p.site_xpos, model.nsite, 3)
        self._site_xmat = _wrap_mjtNum_2d(p.site_xmat, model.nsite, 9)
        self._cam_xpos = _wrap_mjtNum_2d(p.cam_xpos, model.ncam, 3)
        self._cam_xmat = _wrap_mjtNum_2d(p.cam_xmat, model.ncam, 9)
        self._light_xpos = _wrap_mjtNum_2d(p.light_xpos, model.nlight, 3)
        self._light_xdir = _wrap_mjtNum_2d(p.light_xdir, model.nlight, 3)
        self._subtree_com = _wrap_mjtNum_2d(p.subtree_com, model.nbody, 3)
        self._cdof = _wrap_mjtNum_2d(p.cdof, model.nv, 6)
        self._cinert = _wrap_mjtNum_2d(p.cinert, model.nbody, 10)
        self._ten_wrapadr = _wrap_int_1d(p.ten_wrapadr, model.ntendon)
        self._ten_wrapnum = _wrap_int_1d(p.ten_wrapnum, model.ntendon)
        self._ten_length = _wrap_mjtNum_1d(p.ten_length, model.ntendon)
        self._ten_moment = _wrap_mjtNum_2d(p.ten_moment, model.ntendon, model.nv)
        self._wrap_obj = _wrap_int_1d(p.wrap_obj, model.nwrap*2)
        self._wrap_xpos = _wrap_mjtNum_2d(p.wrap_xpos, model.nwrap*2, 3)
        self._actuator_length = _wrap_mjtNum_1d(p.actuator_length, model.nu)
        self._actuator_moment = _wrap_mjtNum_2d(p.actuator_moment, model.nu, model.nv)
        self._crb = _wrap_mjtNum_2d(p.crb, model.nbody, 10)
        self._qM = _wrap_mjtNum_1d(p.qM, model.nM)
        self._qLD = _wrap_mjtNum_1d(p.qLD, model.nM)
        self._qLDiagInv = _wrap_mjtNum_1d(p.qLDiagInv, model.nv)
        self._qLDiagSqrtInv = _wrap_mjtNum_1d(p.qLDiagSqrtInv, model.nv)
        self._contact = tuple([WrapMjContact(&p.contact[i]) for i in range(model.nconmax)])
        self._efc_type = _wrap_int_1d(p.efc_type, model.njmax)
        self._efc_id = _wrap_int_1d(p.efc_id, model.njmax)
        self._efc_J_rownnz = _wrap_int_1d(p.efc_J_rownnz, model.njmax)
        self._efc_J_rowadr = _wrap_int_1d(p.efc_J_rowadr, model.njmax)
        self._efc_J_colind = _wrap_int_2d(p.efc_J_colind, model.njmax, model.nv)
        self._efc_JT_rownnz = _wrap_int_1d(p.efc_JT_rownnz, model.nv)
        self._efc_JT_rowadr = _wrap_int_1d(p.efc_JT_rowadr, model.nv)
        self._efc_JT_colind = _wrap_int_2d(p.efc_JT_colind, model.nv, model.njmax)
        self._efc_solref = _wrap_mjtNum_2d(p.efc_solref, model.njmax, mjNREF)
        self._efc_solimp = _wrap_mjtNum_2d(p.efc_solimp, model.njmax, mjNIMP)
        self._efc_margin = _wrap_mjtNum_1d(p.efc_margin, model.njmax)
        self._efc_frictionloss = _wrap_mjtNum_1d(p.efc_frictionloss, model.njmax)
        self._efc_pos = _wrap_mjtNum_1d(p.efc_pos, model.njmax)
        self._efc_J = _wrap_mjtNum_2d(p.efc_J, model.njmax, model.nv)
        self._efc_JT = _wrap_mjtNum_2d(p.efc_JT, model.nv, model.njmax)
        self._efc_diagApprox = _wrap_mjtNum_1d(p.efc_diagApprox, model.njmax)
        self._efc_D = _wrap_mjtNum_1d(p.efc_D, model.njmax)
        self._efc_R = _wrap_mjtNum_1d(p.efc_R, model.njmax)
        self._efc_AR_rownnz = _wrap_int_1d(p.efc_AR_rownnz, model.njmax)
        self._efc_AR_rowadr = _wrap_int_1d(p.efc_AR_rowadr, model.njmax)
        self._efc_AR_colind = _wrap_int_2d(p.efc_AR_colind, model.njmax, model.njmax)
        self._efc_AR = _wrap_mjtNum_2d(p.efc_AR, model.njmax, model.njmax)
        self._ten_velocity = _wrap_mjtNum_1d(p.ten_velocity, model.ntendon)
        self._actuator_velocity = _wrap_mjtNum_1d(p.actuator_velocity, model.nu)
        self._cvel = _wrap_mjtNum_2d(p.cvel, model.nbody, 6)
        self._cdof_dot = _wrap_mjtNum_2d(p.cdof_dot, model.nv, 6)
        self._qfrc_bias = _wrap_mjtNum_1d(p.qfrc_bias, model.nv)
        self._qfrc_passive = _wrap_mjtNum_1d(p.qfrc_passive, model.nv)
        self._efc_vel = _wrap_mjtNum_1d(p.efc_vel, model.njmax)
        self._efc_aref = _wrap_mjtNum_1d(p.efc_aref, model.njmax)
        self._subtree_linvel = _wrap_mjtNum_2d(p.subtree_linvel, model.nbody, 3)
        self._subtree_angmom = _wrap_mjtNum_2d(p.subtree_angmom, model.nbody, 3)
        self._actuator_force = _wrap_mjtNum_1d(p.actuator_force, model.nu)
        self._qfrc_actuator = _wrap_mjtNum_1d(p.qfrc_actuator, model.nv)
        self._qfrc_unc = _wrap_mjtNum_1d(p.qfrc_unc, model.nv)
        self._qacc_unc = _wrap_mjtNum_1d(p.qacc_unc, model.nv)
        self._efc_b = _wrap_mjtNum_1d(p.efc_b, model.njmax)
        self._efc_force = _wrap_mjtNum_1d(p.efc_force, model.njmax)
        self._efc_state = _wrap_int_1d(p.efc_state, model.njmax)
        self._qfrc_constraint = _wrap_mjtNum_1d(p.qfrc_constraint, model.nv)
        self._qacc_warmstart = _wrap_mjtNum_1d(p.qacc_warmstart, model.nv)
        self._qfrc_inverse = _wrap_mjtNum_1d(p.qfrc_inverse, model.nv)
        self._cacc = _wrap_mjtNum_2d(p.cacc, model.nbody, 6)
        self._cfrc_int = _wrap_mjtNum_2d(p.cfrc_int, model.nbody, 6)
        self._cfrc_ext = _wrap_mjtNum_2d(p.cfrc_ext, model.nbody, 6)
        self._warning = [WrapMjWarningStat(&p.warning[i]) for i in range(mjNWARNING)]
        self._timer = [WrapMjTimerStat(&p.timer[i]) for i in range(mjNTIMER)]
        self._solver = [WrapMjSolverStat(&p.solver[i]) for i in range(1000)]
        self._solver_fwdinv = _wrap_mjtNum_1d(&p.solver_fwdinv[0], 2)
        self._energy = _wrap_mjtNum_1d(&p.energy[0], 2)
        
    @property
    def nstack(self): return self.ptr.nstack
    @nstack.setter
    def nstack(self, int x): self.ptr.nstack = x
    @property
    def nbuffer(self): return self.ptr.nbuffer
    @nbuffer.setter
    def nbuffer(self, int x): self.ptr.nbuffer = x
    @property
    def pstack(self): return self.ptr.pstack
    @pstack.setter
    def pstack(self, int x): self.ptr.pstack = x
    @property
    def maxuse_stack(self): return self.ptr.maxuse_stack
    @maxuse_stack.setter
    def maxuse_stack(self, int x): self.ptr.maxuse_stack = x
    @property
    def maxuse_con(self): return self.ptr.maxuse_con
    @maxuse_con.setter
    def maxuse_con(self, int x): self.ptr.maxuse_con = x
    @property
    def maxuse_efc(self): return self.ptr.maxuse_efc
    @maxuse_efc.setter
    def maxuse_efc(self, int x): self.ptr.maxuse_efc = x
    @property
    def solver_iter(self): return self.ptr.solver_iter
    @solver_iter.setter
    def solver_iter(self, int x): self.ptr.solver_iter = x
    @property
    def solver_nnz(self): return self.ptr.solver_nnz
    @solver_nnz.setter
    def solver_nnz(self, int x): self.ptr.solver_nnz = x
    @property
    def ne(self): return self.ptr.ne
    @ne.setter
    def ne(self, int x): self.ptr.ne = x
    @property
    def nf(self): return self.ptr.nf
    @nf.setter
    def nf(self, int x): self.ptr.nf = x
    @property
    def nefc(self): return self.ptr.nefc
    @nefc.setter
    def nefc(self, int x): self.ptr.nefc = x
    @property
    def ncon(self): return self.ptr.ncon
    @ncon.setter
    def ncon(self, int x): self.ptr.ncon = x
    @property
    def time(self): return self.ptr.time
    @time.setter
    def time(self, mjtNum x): self.ptr.time = x
    @property
    def qpos(self): return self._qpos
    @property
    def qvel(self): return self._qvel
    @property
    def act(self): return self._act
    @property
    def ctrl(self): return self._ctrl
    @property
    def qfrc_applied(self): return self._qfrc_applied
    @property
    def xfrc_applied(self): return self._xfrc_applied
    @property
    def qacc(self): return self._qacc
    @property
    def act_dot(self): return self._act_dot
    @property
    def mocap_pos(self): return self._mocap_pos
    @property
    def mocap_quat(self): return self._mocap_quat
    @property
    def userdata(self): return self._userdata
    @property
    def sensordata(self): return self._sensordata

    @property
    def xpos(self):
        raise RuntimeError("body_xpos should be used instead of xpos")


    @property
    def xquat(self):
        raise RuntimeError("body_xquat should be used instead of xquat")


    @property
    def xmat(self):
        raise RuntimeError("body_xmat should be used instead of xmat")

    @property
    def xipos(self): return self._xipos
    @property
    def ximat(self): return self._ximat
    @property
    def xanchor(self): return self._xanchor
    @property
    def xaxis(self): return self._xaxis
    @property
    def geom_xpos(self): return self._geom_xpos
    @property
    def geom_xmat(self): return self._geom_xmat
    @property
    def site_xpos(self): return self._site_xpos
    @property
    def site_xmat(self): return self._site_xmat
    @property
    def cam_xpos(self): return self._cam_xpos
    @property
    def cam_xmat(self): return self._cam_xmat
    @property
    def light_xpos(self): return self._light_xpos
    @property
    def light_xdir(self): return self._light_xdir
    @property
    def subtree_com(self): return self._subtree_com
    @property
    def cdof(self): return self._cdof
    @property
    def cinert(self): return self._cinert
    @property
    def ten_wrapadr(self): return self._ten_wrapadr
    @property
    def ten_wrapnum(self): return self._ten_wrapnum
    @property
    def ten_length(self): return self._ten_length
    @property
    def ten_moment(self): return self._ten_moment
    @property
    def wrap_obj(self): return self._wrap_obj
    @property
    def wrap_xpos(self): return self._wrap_xpos
    @property
    def actuator_length(self): return self._actuator_length
    @property
    def actuator_moment(self): return self._actuator_moment
    @property
    def crb(self): return self._crb
    @property
    def qM(self): return self._qM
    @property
    def qLD(self): return self._qLD
    @property
    def qLDiagInv(self): return self._qLDiagInv
    @property
    def qLDiagSqrtInv(self): return self._qLDiagSqrtInv
    @property
    def contact(self): return self._contact
    @property
    def efc_type(self): return self._efc_type
    @property
    def efc_id(self): return self._efc_id
    @property
    def efc_J_rownnz(self): return self._efc_J_rownnz
    @property
    def efc_J_rowadr(self): return self._efc_J_rowadr
    @property
    def efc_J_colind(self): return self._efc_J_colind
    @property
    def efc_JT_rownnz(self): return self._efc_JT_rownnz
    @property
    def efc_JT_rowadr(self): return self._efc_JT_rowadr
    @property
    def efc_JT_colind(self): return self._efc_JT_colind
    @property
    def efc_solref(self): return self._efc_solref
    @property
    def efc_solimp(self): return self._efc_solimp
    @property
    def efc_margin(self): return self._efc_margin
    @property
    def efc_frictionloss(self): return self._efc_frictionloss

    @property
    def efc_pos(self):
        raise RuntimeError("active_contacts_efc_pos should be used instead of efc_pos")

    @property
    def efc_J(self): return self._efc_J
    @property
    def efc_JT(self): return self._efc_JT
    @property
    def efc_diagApprox(self): return self._efc_diagApprox
    @property
    def efc_D(self): return self._efc_D
    @property
    def efc_R(self): return self._efc_R
    @property
    def efc_AR_rownnz(self): return self._efc_AR_rownnz
    @property
    def efc_AR_rowadr(self): return self._efc_AR_rowadr
    @property
    def efc_AR_colind(self): return self._efc_AR_colind
    @property
    def efc_AR(self): return self._efc_AR
    @property
    def ten_velocity(self): return self._ten_velocity
    @property
    def actuator_velocity(self): return self._actuator_velocity
    @property
    def cvel(self): return self._cvel
    @property
    def cdof_dot(self): return self._cdof_dot
    @property
    def qfrc_bias(self): return self._qfrc_bias
    @property
    def qfrc_passive(self): return self._qfrc_passive
    @property
    def efc_vel(self): return self._efc_vel
    @property
    def efc_aref(self): return self._efc_aref
    @property
    def subtree_linvel(self): return self._subtree_linvel
    @property
    def subtree_angmom(self): return self._subtree_angmom
    @property
    def actuator_force(self): return self._actuator_force
    @property
    def qfrc_actuator(self): return self._qfrc_actuator
    @property
    def qfrc_unc(self): return self._qfrc_unc
    @property
    def qacc_unc(self): return self._qacc_unc
    @property
    def efc_b(self): return self._efc_b
    @property
    def efc_force(self): return self._efc_force
    @property
    def efc_state(self): return self._efc_state
    @property
    def qfrc_constraint(self): return self._qfrc_constraint
    @property
    def qacc_warmstart(self): return self._qacc_warmstart
    @property
    def qfrc_inverse(self): return self._qfrc_inverse
    @property
    def cacc(self): return self._cacc
    @property
    def cfrc_int(self): return self._cfrc_int
    @property
    def cfrc_ext(self): return self._cfrc_ext
    @property
    def warning(self): return self._warning
    @property
    def timer(self): return self._timer
    @property
    def solver(self): return self._solver
    @property
    def solver_fwdinv(self): return self._solver_fwdinv
    @property
    def energy(self): return self._energy
    @property
    def body_jacp(self):
        jacps = np.zeros((self._model.nbody, 3 * self._model.nv))
        cdef double [:] jacp_view
        for i, jacp in enumerate(jacps):
            jacp_view = jacp
            mj_jacBody(self._model.ptr, self.ptr, &jacp_view[0], NULL, i)
        return jacps

    @property
    def body_jacr(self):
        jacrs = np.zeros((self._model.nbody, 3 * self._model.nv))
        cdef double [:] jacr_view
        for i, jacr in enumerate(jacrs):
            jacr_view = jacr
            mj_jacBody(self._model.ptr, self.ptr, NULL, &jacr_view[0], i)
        return jacrs

    @property
    def body_xvelp(self):
        jacp = self.body_jacp.reshape((self._model.nbody, 3, self._model.nv))
        xvelp = np.dot(jacp, self.qvel)
        return xvelp

    @property
    def body_xvelr(self):
        jacr = self.body_jacr.reshape((self._model.nbody, 3, self._model.nv))
        xvelr = np.dot(jacr, self.qvel)
        return xvelr

    @property
    def geom_jacp(self):
        jacps = np.zeros((self._model.ngeom, 3 * self._model.nv))
        cdef double [:] jacp_view
        for i, jacp in enumerate(jacps):
            jacp_view = jacp
            mj_jacGeom(self._model.ptr, self.ptr, &jacp_view[0], NULL, i)
        return jacps

    @property
    def geom_jacr(self):
        jacrs = np.zeros((self._model.ngeom, 3 * self._model.nv))
        cdef double [:] jacr_view
        for i, jacr in enumerate(jacrs):
            jacr_view = jacr
            mj_jacGeom(self._model.ptr, self.ptr, NULL, &jacr_view[0], i)
        return jacrs

    @property
    def geom_xvelp(self):
        jacp = self.geom_jacp.reshape((self._model.ngeom, 3, self._model.nv))
        xvelp = np.dot(jacp, self.qvel)
        return xvelp

    @property
    def geom_xvelr(self):
        jacr = self.geom_jacr.reshape((self._model.ngeom, 3, self._model.nv))
        xvelr = np.dot(jacr, self.qvel)
        return xvelr

    @property
    def site_jacp(self):
        jacps = np.zeros((self._model.nsite, 3 * self._model.nv))
        cdef double [:] jacp_view
        for i, jacp in enumerate(jacps):
            jacp_view = jacp
            mj_jacSite(self._model.ptr, self.ptr, &jacp_view[0], NULL, i)
        return jacps

    @property
    def site_jacr(self):
        jacrs = np.zeros((self._model.nsite, 3 * self._model.nv))
        cdef double [:] jacr_view
        for i, jacr in enumerate(jacrs):
            jacr_view = jacr
            mj_jacSite(self._model.ptr, self.ptr, NULL, &jacr_view[0], i)
        return jacrs

    @property
    def site_xvelp(self):
        jacp = self.site_jacp.reshape((self._model.nsite, 3, self._model.nv))
        xvelp = np.dot(jacp, self.qvel)
        return xvelp

    @property
    def site_xvelr(self):
        jacr = self.site_jacr.reshape((self._model.nsite, 3, self._model.nv))
        xvelr = np.dot(jacr, self.qvel)
        return xvelr


cdef PyMjData WrapMjData(mjData* p, PyMjModel model):
    cdef PyMjData o = PyMjData()
    o._set(p, model)
    return o

cdef class PyMjVisual_rgba(object):
    cdef mjVisual_rgba* ptr
    
    
    cdef np.ndarray _fog
    cdef np.ndarray _force
    cdef np.ndarray _inertia
    cdef np.ndarray _joint
    cdef np.ndarray _actuator
    cdef np.ndarray _com
    cdef np.ndarray _camera
    cdef np.ndarray _light
    cdef np.ndarray _selectpoint
    cdef np.ndarray _connect
    cdef np.ndarray _contactpoint
    cdef np.ndarray _contactforce
    cdef np.ndarray _contactfriction
    cdef np.ndarray _contacttorque
    cdef np.ndarray _constraint
    cdef np.ndarray _slidercrank
    cdef np.ndarray _crankbroken
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjVisual_rgba* p):
        
        self.ptr = p
        
        
        self._fog = _wrap_float_1d(&p.fog[0], 4)
        self._force = _wrap_float_1d(&p.force[0], 4)
        self._inertia = _wrap_float_1d(&p.inertia[0], 4)
        self._joint = _wrap_float_1d(&p.joint[0], 4)
        self._actuator = _wrap_float_1d(&p.actuator[0], 4)
        self._com = _wrap_float_1d(&p.com[0], 4)
        self._camera = _wrap_float_1d(&p.camera[0], 4)
        self._light = _wrap_float_1d(&p.light[0], 4)
        self._selectpoint = _wrap_float_1d(&p.selectpoint[0], 4)
        self._connect = _wrap_float_1d(&p.connect[0], 4)
        self._contactpoint = _wrap_float_1d(&p.contactpoint[0], 4)
        self._contactforce = _wrap_float_1d(&p.contactforce[0], 4)
        self._contactfriction = _wrap_float_1d(&p.contactfriction[0], 4)
        self._contacttorque = _wrap_float_1d(&p.contacttorque[0], 4)
        self._constraint = _wrap_float_1d(&p.constraint[0], 4)
        self._slidercrank = _wrap_float_1d(&p.slidercrank[0], 4)
        self._crankbroken = _wrap_float_1d(&p.crankbroken[0], 4)
        
    @property
    def fog(self): return self._fog
    @property
    def force(self): return self._force
    @property
    def inertia(self): return self._inertia
    @property
    def joint(self): return self._joint
    @property
    def actuator(self): return self._actuator
    @property
    def com(self): return self._com
    @property
    def camera(self): return self._camera
    @property
    def light(self): return self._light
    @property
    def selectpoint(self): return self._selectpoint
    @property
    def connect(self): return self._connect
    @property
    def contactpoint(self): return self._contactpoint
    @property
    def contactforce(self): return self._contactforce
    @property
    def contactfriction(self): return self._contactfriction
    @property
    def contacttorque(self): return self._contacttorque
    @property
    def constraint(self): return self._constraint
    @property
    def slidercrank(self): return self._slidercrank
    @property
    def crankbroken(self): return self._crankbroken

cdef PyMjVisual_rgba WrapMjVisual_rgba(mjVisual_rgba* p):
    cdef PyMjVisual_rgba o = PyMjVisual_rgba()
    o._set(p)
    return o

cdef class PyMjTimerStat(object):
    cdef mjTimerStat* ptr
    
    
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjTimerStat* p):
        
        self.ptr = p
        
        
        
    @property
    def duration(self): return self.ptr.duration
    @duration.setter
    def duration(self, mjtNum x): self.ptr.duration = x
    @property
    def number(self): return self.ptr.number
    @number.setter
    def number(self, int x): self.ptr.number = x

cdef PyMjTimerStat WrapMjTimerStat(mjTimerStat* p):
    cdef PyMjTimerStat o = PyMjTimerStat()
    o._set(p)
    return o

cdef class PyMjvGeom(object):
    cdef mjvGeom* ptr
    
    
    cdef np.ndarray _texrepeat
    cdef np.ndarray _size
    cdef np.ndarray _pos
    cdef np.ndarray _mat
    cdef np.ndarray _rgba
    cdef np.ndarray _label
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjvGeom* p):
        
        self.ptr = p
        
        
        self._texrepeat = _wrap_float_1d(&p.texrepeat[0], 2)
        self._size = _wrap_float_1d(&p.size[0], 3)
        self._pos = _wrap_float_1d(&p.pos[0], 3)
        self._mat = _wrap_float_1d(&p.mat[0], 9)
        self._rgba = _wrap_float_1d(&p.rgba[0], 4)
        self._label = _wrap_char_1d(&p.label[0], 100)
        
    @property
    def type(self): return self.ptr.type
    @type.setter
    def type(self, int x): self.ptr.type = x
    @property
    def dataid(self): return self.ptr.dataid
    @dataid.setter
    def dataid(self, int x): self.ptr.dataid = x
    @property
    def objtype(self): return self.ptr.objtype
    @objtype.setter
    def objtype(self, int x): self.ptr.objtype = x
    @property
    def objid(self): return self.ptr.objid
    @objid.setter
    def objid(self, int x): self.ptr.objid = x
    @property
    def category(self): return self.ptr.category
    @category.setter
    def category(self, int x): self.ptr.category = x
    @property
    def texid(self): return self.ptr.texid
    @texid.setter
    def texid(self, int x): self.ptr.texid = x
    @property
    def texuniform(self): return self.ptr.texuniform
    @texuniform.setter
    def texuniform(self, int x): self.ptr.texuniform = x
    @property
    def emission(self): return self.ptr.emission
    @emission.setter
    def emission(self, float x): self.ptr.emission = x
    @property
    def specular(self): return self.ptr.specular
    @specular.setter
    def specular(self, float x): self.ptr.specular = x
    @property
    def shininess(self): return self.ptr.shininess
    @shininess.setter
    def shininess(self, float x): self.ptr.shininess = x
    @property
    def reflectance(self): return self.ptr.reflectance
    @reflectance.setter
    def reflectance(self, float x): self.ptr.reflectance = x
    @property
    def camdist(self): return self.ptr.camdist
    @camdist.setter
    def camdist(self, float x): self.ptr.camdist = x
    @property
    def modelrbound(self): return self.ptr.modelrbound
    @modelrbound.setter
    def modelrbound(self, float x): self.ptr.modelrbound = x
    @property
    def transparent(self): return self.ptr.transparent
    @transparent.setter
    def transparent(self, mjtByte x): self.ptr.transparent = x
    @property
    def texrepeat(self): return self._texrepeat
    @property
    def size(self): return self._size
    @property
    def pos(self): return self._pos
    @property
    def mat(self): return self._mat
    @property
    def rgba(self): return self._rgba
    @property
    def label(self): return self._label

cdef PyMjvGeom WrapMjvGeom(mjvGeom* p):
    cdef PyMjvGeom o = PyMjvGeom()
    o._set(p)
    return o

cdef class PyMjVisual_global_(object):
    cdef mjVisual_global_* ptr
    
    
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjVisual_global_* p):
        
        self.ptr = p
        
        
        
    @property
    def fovy(self): return self.ptr.fovy
    @fovy.setter
    def fovy(self, float x): self.ptr.fovy = x
    @property
    def ipd(self): return self.ptr.ipd
    @ipd.setter
    def ipd(self, float x): self.ptr.ipd = x
    @property
    def linewidth(self): return self.ptr.linewidth
    @linewidth.setter
    def linewidth(self, float x): self.ptr.linewidth = x
    @property
    def glow(self): return self.ptr.glow
    @glow.setter
    def glow(self, float x): self.ptr.glow = x
    @property
    def offwidth(self): return self.ptr.offwidth
    @offwidth.setter
    def offwidth(self, int x): self.ptr.offwidth = x
    @property
    def offheight(self): return self.ptr.offheight
    @offheight.setter
    def offheight(self, int x): self.ptr.offheight = x

cdef PyMjVisual_global_ WrapMjVisual_global_(mjVisual_global_* p):
    cdef PyMjVisual_global_ o = PyMjVisual_global_()
    o._set(p)
    return o

cdef class PyMjrContext(object):
    cdef mjrContext* ptr
    
    
    cdef np.ndarray _textureType
    cdef np.ndarray _texture
    cdef np.ndarray _charWidth
    cdef np.ndarray _charWidthBig
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjrContext* p):
        
        self.ptr = p
        
        
        self._textureType = _wrap_int_1d(&p.textureType[0], 100)
        self._texture = _wrap_unsigned_int_1d(&p.texture[0], 100)
        self._charWidth = _wrap_int_1d(&p.charWidth[0], 127)
        self._charWidthBig = _wrap_int_1d(&p.charWidthBig[0], 127)
        
    @property
    def lineWidth(self): return self.ptr.lineWidth
    @lineWidth.setter
    def lineWidth(self, float x): self.ptr.lineWidth = x
    @property
    def shadowClip(self): return self.ptr.shadowClip
    @shadowClip.setter
    def shadowClip(self, float x): self.ptr.shadowClip = x
    @property
    def shadowScale(self): return self.ptr.shadowScale
    @shadowScale.setter
    def shadowScale(self, float x): self.ptr.shadowScale = x
    @property
    def shadowSize(self): return self.ptr.shadowSize
    @shadowSize.setter
    def shadowSize(self, int x): self.ptr.shadowSize = x
    @property
    def offWidth(self): return self.ptr.offWidth
    @offWidth.setter
    def offWidth(self, int x): self.ptr.offWidth = x
    @property
    def offHeight(self): return self.ptr.offHeight
    @offHeight.setter
    def offHeight(self, int x): self.ptr.offHeight = x
    @property
    def offSamples(self): return self.ptr.offSamples
    @offSamples.setter
    def offSamples(self, int x): self.ptr.offSamples = x
    @property
    def offFBO(self): return self.ptr.offFBO
    @offFBO.setter
    def offFBO(self, unsigned int x): self.ptr.offFBO = x
    @property
    def offFBO_r(self): return self.ptr.offFBO_r
    @offFBO_r.setter
    def offFBO_r(self, unsigned int x): self.ptr.offFBO_r = x
    @property
    def offColor(self): return self.ptr.offColor
    @offColor.setter
    def offColor(self, unsigned int x): self.ptr.offColor = x
    @property
    def offColor_r(self): return self.ptr.offColor_r
    @offColor_r.setter
    def offColor_r(self, unsigned int x): self.ptr.offColor_r = x
    @property
    def offDepthStencil(self): return self.ptr.offDepthStencil
    @offDepthStencil.setter
    def offDepthStencil(self, unsigned int x): self.ptr.offDepthStencil = x
    @property
    def offDepthStencil_r(self): return self.ptr.offDepthStencil_r
    @offDepthStencil_r.setter
    def offDepthStencil_r(self, unsigned int x): self.ptr.offDepthStencil_r = x
    @property
    def shadowFBO(self): return self.ptr.shadowFBO
    @shadowFBO.setter
    def shadowFBO(self, unsigned int x): self.ptr.shadowFBO = x
    @property
    def shadowTex(self): return self.ptr.shadowTex
    @shadowTex.setter
    def shadowTex(self, unsigned int x): self.ptr.shadowTex = x
    @property
    def ntexture(self): return self.ptr.ntexture
    @ntexture.setter
    def ntexture(self, int x): self.ptr.ntexture = x
    @property
    def basePlane(self): return self.ptr.basePlane
    @basePlane.setter
    def basePlane(self, unsigned int x): self.ptr.basePlane = x
    @property
    def baseMesh(self): return self.ptr.baseMesh
    @baseMesh.setter
    def baseMesh(self, unsigned int x): self.ptr.baseMesh = x
    @property
    def baseHField(self): return self.ptr.baseHField
    @baseHField.setter
    def baseHField(self, unsigned int x): self.ptr.baseHField = x
    @property
    def baseBuiltin(self): return self.ptr.baseBuiltin
    @baseBuiltin.setter
    def baseBuiltin(self, unsigned int x): self.ptr.baseBuiltin = x
    @property
    def baseFontNormal(self): return self.ptr.baseFontNormal
    @baseFontNormal.setter
    def baseFontNormal(self, unsigned int x): self.ptr.baseFontNormal = x
    @property
    def baseFontShadow(self): return self.ptr.baseFontShadow
    @baseFontShadow.setter
    def baseFontShadow(self, unsigned int x): self.ptr.baseFontShadow = x
    @property
    def baseFontBig(self): return self.ptr.baseFontBig
    @baseFontBig.setter
    def baseFontBig(self, unsigned int x): self.ptr.baseFontBig = x
    @property
    def rangePlane(self): return self.ptr.rangePlane
    @rangePlane.setter
    def rangePlane(self, int x): self.ptr.rangePlane = x
    @property
    def rangeMesh(self): return self.ptr.rangeMesh
    @rangeMesh.setter
    def rangeMesh(self, int x): self.ptr.rangeMesh = x
    @property
    def rangeHField(self): return self.ptr.rangeHField
    @rangeHField.setter
    def rangeHField(self, int x): self.ptr.rangeHField = x
    @property
    def rangeBuiltin(self): return self.ptr.rangeBuiltin
    @rangeBuiltin.setter
    def rangeBuiltin(self, int x): self.ptr.rangeBuiltin = x
    @property
    def rangeFont(self): return self.ptr.rangeFont
    @rangeFont.setter
    def rangeFont(self, int x): self.ptr.rangeFont = x
    @property
    def charHeight(self): return self.ptr.charHeight
    @charHeight.setter
    def charHeight(self, int x): self.ptr.charHeight = x
    @property
    def charHeightBig(self): return self.ptr.charHeightBig
    @charHeightBig.setter
    def charHeightBig(self, int x): self.ptr.charHeightBig = x
    @property
    def glewInitialized(self): return self.ptr.glewInitialized
    @glewInitialized.setter
    def glewInitialized(self, int x): self.ptr.glewInitialized = x
    @property
    def windowAvailable(self): return self.ptr.windowAvailable
    @windowAvailable.setter
    def windowAvailable(self, int x): self.ptr.windowAvailable = x
    @property
    def windowSamples(self): return self.ptr.windowSamples
    @windowSamples.setter
    def windowSamples(self, int x): self.ptr.windowSamples = x
    @property
    def windowStereo(self): return self.ptr.windowStereo
    @windowStereo.setter
    def windowStereo(self, int x): self.ptr.windowStereo = x
    @property
    def windowDoublebuffer(self): return self.ptr.windowDoublebuffer
    @windowDoublebuffer.setter
    def windowDoublebuffer(self, int x): self.ptr.windowDoublebuffer = x
    @property
    def currentBuffer(self): return self.ptr.currentBuffer
    @currentBuffer.setter
    def currentBuffer(self, int x): self.ptr.currentBuffer = x
    @property
    def textureType(self): return self._textureType
    @property
    def texture(self): return self._texture
    @property
    def charWidth(self): return self._charWidth
    @property
    def charWidthBig(self): return self._charWidthBig

cdef PyMjrContext WrapMjrContext(mjrContext* p):
    cdef PyMjrContext o = PyMjrContext()
    o._set(p)
    return o

cdef class PyMjVFS(object):
    cdef mjVFS* ptr
    
    
    cdef np.ndarray _filesize
    
    def __cinit__(self):
        self.ptr = <mjVFS*> PyMem_Malloc(sizeof(mjVFS))
        if not self.ptr:
            raise MemoryError()

    def __dealloc__(self):
        PyMem_Free(self.ptr)

    cdef void _set(self, mjVFS* p):
        
        self.ptr = p
        
        
        self._filesize = _wrap_int_1d(&p.filesize[0], 200)
        
    @property
    def nfile(self): return self.ptr.nfile
    @nfile.setter
    def nfile(self, int x): self.ptr.nfile = x
    @property
    def filesize(self): return self._filesize

cdef PyMjVFS WrapMjVFS(mjVFS* p):
    cdef PyMjVFS o = PyMjVFS()
    o._set(p)
    return o

cdef class PyMjVisual_scale(object):
    cdef mjVisual_scale* ptr
    
    
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjVisual_scale* p):
        
        self.ptr = p
        
        
        
    @property
    def forcewidth(self): return self.ptr.forcewidth
    @forcewidth.setter
    def forcewidth(self, float x): self.ptr.forcewidth = x
    @property
    def contactwidth(self): return self.ptr.contactwidth
    @contactwidth.setter
    def contactwidth(self, float x): self.ptr.contactwidth = x
    @property
    def contactheight(self): return self.ptr.contactheight
    @contactheight.setter
    def contactheight(self, float x): self.ptr.contactheight = x
    @property
    def connect(self): return self.ptr.connect
    @connect.setter
    def connect(self, float x): self.ptr.connect = x
    @property
    def com(self): return self.ptr.com
    @com.setter
    def com(self, float x): self.ptr.com = x
    @property
    def camera(self): return self.ptr.camera
    @camera.setter
    def camera(self, float x): self.ptr.camera = x
    @property
    def light(self): return self.ptr.light
    @light.setter
    def light(self, float x): self.ptr.light = x
    @property
    def selectpoint(self): return self.ptr.selectpoint
    @selectpoint.setter
    def selectpoint(self, float x): self.ptr.selectpoint = x
    @property
    def jointlength(self): return self.ptr.jointlength
    @jointlength.setter
    def jointlength(self, float x): self.ptr.jointlength = x
    @property
    def jointwidth(self): return self.ptr.jointwidth
    @jointwidth.setter
    def jointwidth(self, float x): self.ptr.jointwidth = x
    @property
    def actuatorlength(self): return self.ptr.actuatorlength
    @actuatorlength.setter
    def actuatorlength(self, float x): self.ptr.actuatorlength = x
    @property
    def actuatorwidth(self): return self.ptr.actuatorwidth
    @actuatorwidth.setter
    def actuatorwidth(self, float x): self.ptr.actuatorwidth = x
    @property
    def framelength(self): return self.ptr.framelength
    @framelength.setter
    def framelength(self, float x): self.ptr.framelength = x
    @property
    def framewidth(self): return self.ptr.framewidth
    @framewidth.setter
    def framewidth(self, float x): self.ptr.framewidth = x
    @property
    def constraint(self): return self.ptr.constraint
    @constraint.setter
    def constraint(self, float x): self.ptr.constraint = x
    @property
    def slidercrank(self): return self.ptr.slidercrank
    @slidercrank.setter
    def slidercrank(self, float x): self.ptr.slidercrank = x

cdef PyMjVisual_scale WrapMjVisual_scale(mjVisual_scale* p):
    cdef PyMjVisual_scale o = PyMjVisual_scale()
    o._set(p)
    return o

cdef class PyMjStatistic(object):
    cdef mjStatistic* ptr
    
    
    cdef np.ndarray _center
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjStatistic* p):
        
        self.ptr = p
        
        
        self._center = _wrap_mjtNum_1d(&p.center[0], 3)
        
    @property
    def meaninertia(self): return self.ptr.meaninertia
    @meaninertia.setter
    def meaninertia(self, mjtNum x): self.ptr.meaninertia = x
    @property
    def meanmass(self): return self.ptr.meanmass
    @meanmass.setter
    def meanmass(self, mjtNum x): self.ptr.meanmass = x
    @property
    def meansize(self): return self.ptr.meansize
    @meansize.setter
    def meansize(self, mjtNum x): self.ptr.meansize = x
    @property
    def extent(self): return self.ptr.extent
    @extent.setter
    def extent(self, mjtNum x): self.ptr.extent = x
    @property
    def center(self): return self._center

cdef PyMjStatistic WrapMjStatistic(mjStatistic* p):
    cdef PyMjStatistic o = PyMjStatistic()
    o._set(p)
    return o

cdef class PyMjvScene(object):
    cdef mjvScene* ptr
    
    
    cdef list _lights
    cdef list _camera
    cdef np.ndarray _translate
    cdef np.ndarray _rotate
    cdef np.ndarray _flags
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjvScene* p):
        
        self.ptr = p
        
        
        self._lights = [WrapMjvLight(&p.lights[i]) for i in range(8)]
        self._camera = [WrapMjvGLCamera(&p.camera[i]) for i in range(2)]
        self._translate = _wrap_float_1d(&p.translate[0], 3)
        self._rotate = _wrap_float_1d(&p.rotate[0], 4)
        self._flags = _wrap_mjtByte_1d(&p.flags[0], mjNRNDFLAG)
        
    @property
    def maxgeom(self): return self.ptr.maxgeom
    @maxgeom.setter
    def maxgeom(self, int x): self.ptr.maxgeom = x
    @property
    def ngeom(self): return self.ptr.ngeom
    @ngeom.setter
    def ngeom(self, int x): self.ptr.ngeom = x
    @property
    def nlight(self): return self.ptr.nlight
    @nlight.setter
    def nlight(self, int x): self.ptr.nlight = x
    @property
    def enabletransform(self): return self.ptr.enabletransform
    @enabletransform.setter
    def enabletransform(self, mjtByte x): self.ptr.enabletransform = x
    @property
    def scale(self): return self.ptr.scale
    @scale.setter
    def scale(self, float x): self.ptr.scale = x
    @property
    def stereo(self): return self.ptr.stereo
    @stereo.setter
    def stereo(self, int x): self.ptr.stereo = x
    @property
    def lights(self): return self._lights
    @property
    def camera(self): return self._camera
    @property
    def translate(self): return self._translate
    @property
    def rotate(self): return self._rotate
    @property
    def flags(self): return self._flags

cdef PyMjvScene WrapMjvScene(mjvScene* p):
    cdef PyMjvScene o = PyMjvScene()
    o._set(p)
    return o

cdef class PyMjvLight(object):
    cdef mjvLight* ptr
    
    
    cdef np.ndarray _pos
    cdef np.ndarray _dir
    cdef np.ndarray _attenuation
    cdef np.ndarray _ambient
    cdef np.ndarray _diffuse
    cdef np.ndarray _specular
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjvLight* p):
        
        self.ptr = p
        
        
        self._pos = _wrap_float_1d(&p.pos[0], 3)
        self._dir = _wrap_float_1d(&p.dir[0], 3)
        self._attenuation = _wrap_float_1d(&p.attenuation[0], 3)
        self._ambient = _wrap_float_1d(&p.ambient[0], 3)
        self._diffuse = _wrap_float_1d(&p.diffuse[0], 3)
        self._specular = _wrap_float_1d(&p.specular[0], 3)
        
    @property
    def cutoff(self): return self.ptr.cutoff
    @cutoff.setter
    def cutoff(self, float x): self.ptr.cutoff = x
    @property
    def exponent(self): return self.ptr.exponent
    @exponent.setter
    def exponent(self, float x): self.ptr.exponent = x
    @property
    def headlight(self): return self.ptr.headlight
    @headlight.setter
    def headlight(self, mjtByte x): self.ptr.headlight = x
    @property
    def directional(self): return self.ptr.directional
    @directional.setter
    def directional(self, mjtByte x): self.ptr.directional = x
    @property
    def castshadow(self): return self.ptr.castshadow
    @castshadow.setter
    def castshadow(self, mjtByte x): self.ptr.castshadow = x
    @property
    def pos(self): return self._pos
    @property
    def dir(self): return self._dir
    @property
    def attenuation(self): return self._attenuation
    @property
    def ambient(self): return self._ambient
    @property
    def diffuse(self): return self._diffuse
    @property
    def specular(self): return self._specular

cdef PyMjvLight WrapMjvLight(mjvLight* p):
    cdef PyMjvLight o = PyMjvLight()
    o._set(p)
    return o

cdef class PyMjContact(object):
    cdef mjContact* ptr
    
    
    cdef np.ndarray _pos
    cdef np.ndarray _frame
    cdef np.ndarray _friction
    cdef np.ndarray _solref
    cdef np.ndarray _solimp
    cdef np.ndarray _H
    
    def __cinit__(self):
        self.ptr = NULL

    cdef void _set(self, mjContact* p):
        
        self.ptr = p
        
        
        self._pos = _wrap_mjtNum_1d(&p.pos[0], 3)
        self._frame = _wrap_mjtNum_1d(&p.frame[0], 9)
        self._friction = _wrap_mjtNum_1d(&p.friction[0], 5)
        self._solref = _wrap_mjtNum_1d(&p.solref[0], 2)
        self._solimp = _wrap_mjtNum_1d(&p.solimp[0], 3)
        self._H = _wrap_mjtNum_1d(&p.H[0], 36)
        
    @property
    def dist(self): return self.ptr.dist
    @dist.setter
    def dist(self, mjtNum x): self.ptr.dist = x
    @property
    def includemargin(self): return self.ptr.includemargin
    @includemargin.setter
    def includemargin(self, mjtNum x): self.ptr.includemargin = x
    @property
    def mu(self): return self.ptr.mu
    @mu.setter
    def mu(self, mjtNum x): self.ptr.mu = x
    @property
    def dim(self): return self.ptr.dim
    @dim.setter
    def dim(self, int x): self.ptr.dim = x
    @property
    def geom1(self): return self.ptr.geom1
    @geom1.setter
    def geom1(self, int x): self.ptr.geom1 = x
    @property
    def geom2(self): return self.ptr.geom2
    @geom2.setter
    def geom2(self, int x): self.ptr.geom2 = x
    @property
    def exclude(self): return self.ptr.exclude
    @exclude.setter
    def exclude(self, int x): self.ptr.exclude = x
    @property
    def efc_address(self): return self.ptr.efc_address
    @efc_address.setter
    def efc_address(self, int x): self.ptr.efc_address = x
    @property
    def pos(self): return self._pos
    @property
    def frame(self): return self._frame
    @property
    def friction(self): return self._friction
    @property
    def solref(self): return self._solref
    @property
    def solimp(self): return self._solimp
    @property
    def H(self): return self._H

cdef PyMjContact WrapMjContact(mjContact* p):
    cdef PyMjContact o = PyMjContact()
    o._set(p)
    return o

cdef class PyMjModel(object):
    cdef mjModel* ptr
    
    
    cdef PyMjOption _opt
    cdef PyMjVisual _vis
    cdef PyMjStatistic _stat
    cdef np.ndarray _qpos0
    cdef np.ndarray _qpos_spring
    cdef np.ndarray _body_parentid
    cdef np.ndarray _body_rootid
    cdef np.ndarray _body_weldid
    cdef np.ndarray _body_mocapid
    cdef np.ndarray _body_jntnum
    cdef np.ndarray _body_jntadr
    cdef np.ndarray _body_dofnum
    cdef np.ndarray _body_dofadr
    cdef np.ndarray _body_geomnum
    cdef np.ndarray _body_geomadr
    cdef np.ndarray _body_pos
    cdef np.ndarray _body_quat
    cdef np.ndarray _body_ipos
    cdef np.ndarray _body_iquat
    cdef np.ndarray _body_mass
    cdef np.ndarray _body_subtreemass
    cdef np.ndarray _body_inertia
    cdef np.ndarray _body_invweight0
    cdef np.ndarray _body_user
    cdef np.ndarray _jnt_type
    cdef np.ndarray _jnt_qposadr
    cdef np.ndarray _jnt_dofadr
    cdef np.ndarray _jnt_bodyid
    cdef np.ndarray _jnt_limited
    cdef np.ndarray _jnt_solref
    cdef np.ndarray _jnt_solimp
    cdef np.ndarray _jnt_pos
    cdef np.ndarray _jnt_axis
    cdef np.ndarray _jnt_stiffness
    cdef np.ndarray _jnt_range
    cdef np.ndarray _jnt_margin
    cdef np.ndarray _jnt_user
    cdef np.ndarray _dof_bodyid
    cdef np.ndarray _dof_jntid
    cdef np.ndarray _dof_parentid
    cdef np.ndarray _dof_Madr
    cdef np.ndarray _dof_solref
    cdef np.ndarray _dof_solimp
    cdef np.ndarray _dof_frictionloss
    cdef np.ndarray _dof_armature
    cdef np.ndarray _dof_damping
    cdef np.ndarray _dof_invweight0
    cdef np.ndarray _geom_type
    cdef np.ndarray _geom_contype
    cdef np.ndarray _geom_conaffinity
    cdef np.ndarray _geom_condim
    cdef np.ndarray _geom_bodyid
    cdef np.ndarray _geom_dataid
    cdef np.ndarray _geom_matid
    cdef np.ndarray _geom_group
    cdef np.ndarray _geom_solmix
    cdef np.ndarray _geom_solref
    cdef np.ndarray _geom_solimp
    cdef np.ndarray _geom_size
    cdef np.ndarray _geom_rbound
    cdef np.ndarray _geom_pos
    cdef np.ndarray _geom_quat
    cdef np.ndarray _geom_friction
    cdef np.ndarray _geom_margin
    cdef np.ndarray _geom_gap
    cdef np.ndarray _geom_user
    cdef np.ndarray _geom_rgba
    cdef np.ndarray _site_type
    cdef np.ndarray _site_bodyid
    cdef np.ndarray _site_matid
    cdef np.ndarray _site_group
    cdef np.ndarray _site_size
    cdef np.ndarray _site_pos
    cdef np.ndarray _site_quat
    cdef np.ndarray _site_user
    cdef np.ndarray _site_rgba
    cdef np.ndarray _cam_mode
    cdef np.ndarray _cam_bodyid
    cdef np.ndarray _cam_targetbodyid
    cdef np.ndarray _cam_pos
    cdef np.ndarray _cam_quat
    cdef np.ndarray _cam_poscom0
    cdef np.ndarray _cam_pos0
    cdef np.ndarray _cam_mat0
    cdef np.ndarray _cam_fovy
    cdef np.ndarray _cam_ipd
    cdef np.ndarray _cam_user
    cdef np.ndarray _light_mode
    cdef np.ndarray _light_bodyid
    cdef np.ndarray _light_targetbodyid
    cdef np.ndarray _light_directional
    cdef np.ndarray _light_castshadow
    cdef np.ndarray _light_active
    cdef np.ndarray _light_pos
    cdef np.ndarray _light_dir
    cdef np.ndarray _light_poscom0
    cdef np.ndarray _light_pos0
    cdef np.ndarray _light_dir0
    cdef np.ndarray _light_attenuation
    cdef np.ndarray _light_cutoff
    cdef np.ndarray _light_exponent
    cdef np.ndarray _light_ambient
    cdef np.ndarray _light_diffuse
    cdef np.ndarray _light_specular
    cdef np.ndarray _mesh_faceadr
    cdef np.ndarray _mesh_facenum
    cdef np.ndarray _mesh_vertadr
    cdef np.ndarray _mesh_vertnum
    cdef np.ndarray _mesh_graphadr
    cdef np.ndarray _mesh_vert
    cdef np.ndarray _mesh_normal
    cdef np.ndarray _mesh_face
    cdef np.ndarray _mesh_graph
    cdef np.ndarray _hfield_size
    cdef np.ndarray _hfield_nrow
    cdef np.ndarray _hfield_ncol
    cdef np.ndarray _hfield_adr
    cdef np.ndarray _hfield_data
    cdef np.ndarray _tex_type
    cdef np.ndarray _tex_height
    cdef np.ndarray _tex_width
    cdef np.ndarray _tex_adr
    cdef np.ndarray _tex_rgb
    cdef np.ndarray _mat_texid
    cdef np.ndarray _mat_texuniform
    cdef np.ndarray _mat_texrepeat
    cdef np.ndarray _mat_emission
    cdef np.ndarray _mat_specular
    cdef np.ndarray _mat_shininess
    cdef np.ndarray _mat_reflectance
    cdef np.ndarray _mat_rgba
    cdef np.ndarray _pair_dim
    cdef np.ndarray _pair_geom1
    cdef np.ndarray _pair_geom2
    cdef np.ndarray _pair_signature
    cdef np.ndarray _pair_solref
    cdef np.ndarray _pair_solimp
    cdef np.ndarray _pair_margin
    cdef np.ndarray _pair_gap
    cdef np.ndarray _pair_friction
    cdef np.ndarray _exclude_signature
    cdef np.ndarray _eq_type
    cdef np.ndarray _eq_obj1id
    cdef np.ndarray _eq_obj2id
    cdef np.ndarray _eq_active
    cdef np.ndarray _eq_solref
    cdef np.ndarray _eq_solimp
    cdef np.ndarray _eq_data
    cdef np.ndarray _tendon_adr
    cdef np.ndarray _tendon_num
    cdef np.ndarray _tendon_matid
    cdef np.ndarray _tendon_limited
    cdef np.ndarray _tendon_width
    cdef np.ndarray _tendon_solref_lim
    cdef np.ndarray _tendon_solimp_lim
    cdef np.ndarray _tendon_solref_fri
    cdef np.ndarray _tendon_solimp_fri
    cdef np.ndarray _tendon_range
    cdef np.ndarray _tendon_margin
    cdef np.ndarray _tendon_stiffness
    cdef np.ndarray _tendon_damping
    cdef np.ndarray _tendon_frictionloss
    cdef np.ndarray _tendon_lengthspring
    cdef np.ndarray _tendon_length0
    cdef np.ndarray _tendon_invweight0
    cdef np.ndarray _tendon_user
    cdef np.ndarray _tendon_rgba
    cdef np.ndarray _wrap_type
    cdef np.ndarray _wrap_objid
    cdef np.ndarray _wrap_prm
    cdef np.ndarray _actuator_trntype
    cdef np.ndarray _actuator_dyntype
    cdef np.ndarray _actuator_gaintype
    cdef np.ndarray _actuator_biastype
    cdef np.ndarray _actuator_trnid
    cdef np.ndarray _actuator_ctrllimited
    cdef np.ndarray _actuator_forcelimited
    cdef np.ndarray _actuator_dynprm
    cdef np.ndarray _actuator_gainprm
    cdef np.ndarray _actuator_biasprm
    cdef np.ndarray _actuator_ctrlrange
    cdef np.ndarray _actuator_forcerange
    cdef np.ndarray _actuator_gear
    cdef np.ndarray _actuator_cranklength
    cdef np.ndarray _actuator_invweight0
    cdef np.ndarray _actuator_length0
    cdef np.ndarray _actuator_lengthrange
    cdef np.ndarray _actuator_user
    cdef np.ndarray _sensor_type
    cdef np.ndarray _sensor_datatype
    cdef np.ndarray _sensor_needstage
    cdef np.ndarray _sensor_objtype
    cdef np.ndarray _sensor_objid
    cdef np.ndarray _sensor_dim
    cdef np.ndarray _sensor_adr
    cdef np.ndarray _sensor_cutoff
    cdef np.ndarray _sensor_noise
    cdef np.ndarray _sensor_user
    cdef np.ndarray _numeric_adr
    cdef np.ndarray _numeric_size
    cdef np.ndarray _numeric_data
    cdef np.ndarray _text_adr
    cdef np.ndarray _text_size
    cdef np.ndarray _text_data
    cdef np.ndarray _tuple_adr
    cdef np.ndarray _tuple_size
    cdef np.ndarray _tuple_objtype
    cdef np.ndarray _tuple_objid
    cdef np.ndarray _tuple_objprm
    cdef np.ndarray _key_time
    cdef np.ndarray _key_qpos
    cdef np.ndarray _key_qvel
    cdef np.ndarray _key_act
    cdef np.ndarray _name_bodyadr
    cdef np.ndarray _name_jntadr
    cdef np.ndarray _name_geomadr
    cdef np.ndarray _name_siteadr
    cdef np.ndarray _name_camadr
    cdef np.ndarray _name_lightadr
    cdef np.ndarray _name_meshadr
    cdef np.ndarray _name_hfieldadr
    cdef np.ndarray _name_texadr
    cdef np.ndarray _name_matadr
    cdef np.ndarray _name_eqadr
    cdef np.ndarray _name_tendonadr
    cdef np.ndarray _name_actuatoradr
    cdef np.ndarray _name_sensoradr
    cdef np.ndarray _name_numericadr
    cdef np.ndarray _name_textadr
    cdef np.ndarray _name_tupleadr
    cdef np.ndarray _names
    
    cdef readonly tuple body_names, joint_names, geom_names, site_names, light_names, camera_names, actuator_names, sensor_names
    cdef readonly dict _body_id2name, _joint_id2name, _geom_id2name, _site_id2name, _light_id2name, _camera_id2name, _actuator_id2name, _sensor_id2name
    cdef readonly dict _body_name2id, _joint_name2id, _geom_name2id, _site_name2id, _light_name2id, _camera_name2id, _actuator_name2id, _sensor_name2id

    def body_id2name(self, id):
        if id not in self._body_id2name:
            raise ValueError("No body with id %d exists." % id)
        return self._body_id2name[id]

    def body_name2id(self, name):
        if name not in self._body_name2id:
            raise ValueError("No \"body\" with name %s exists. Available \"body\" names = %s." % (name, self.body_names))
        return self._body_name2id[name]

    def joint_id2name(self, id):
        if id not in self._joint_id2name:
            raise ValueError("No joint with id %d exists." % id)
        return self._joint_id2name[id]

    def joint_name2id(self, name):
        if name not in self._joint_name2id:
            raise ValueError("No \"joint\" with name %s exists. Available \"joint\" names = %s." % (name, self.joint_names))
        return self._joint_name2id[name]

    def geom_id2name(self, id):
        if id not in self._geom_id2name:
            raise ValueError("No geom with id %d exists." % id)
        return self._geom_id2name[id]

    def geom_name2id(self, name):
        if name not in self._geom_name2id:
            raise ValueError("No \"geom\" with name %s exists. Available \"geom\" names = %s." % (name, self.geom_names))
        return self._geom_name2id[name]

    def site_id2name(self, id):
        if id not in self._site_id2name:
            raise ValueError("No site with id %d exists." % id)
        return self._site_id2name[id]

    def site_name2id(self, name):
        if name not in self._site_name2id:
            raise ValueError("No \"site\" with name %s exists. Available \"site\" names = %s." % (name, self.site_names))
        return self._site_name2id[name]

    def light_id2name(self, id):
        if id not in self._light_id2name:
            raise ValueError("No light with id %d exists." % id)
        return self._light_id2name[id]

    def light_name2id(self, name):
        if name not in self._light_name2id:
            raise ValueError("No \"light\" with name %s exists. Available \"light\" names = %s." % (name, self.light_names))
        return self._light_name2id[name]

    def camera_id2name(self, id):
        if id not in self._camera_id2name:
            raise ValueError("No camera with id %d exists." % id)
        return self._camera_id2name[id]

    def camera_name2id(self, name):
        if name not in self._camera_name2id:
            raise ValueError("No \"camera\" with name %s exists. Available \"camera\" names = %s." % (name, self.camera_names))
        return self._camera_name2id[name]

    def actuator_id2name(self, id):
        if id not in self._actuator_id2name:
            raise ValueError("No actuator with id %d exists." % id)
        return self._actuator_id2name[id]

    def actuator_name2id(self, name):
        if name not in self._actuator_name2id:
            raise ValueError("No \"actuator\" with name %s exists. Available \"actuator\" names = %s." % (name, self.actuator_names))
        return self._actuator_name2id[name]

    def sensor_id2name(self, id):
        if id not in self._sensor_id2name:
            raise ValueError("No sensor with id %d exists." % id)
        return self._sensor_id2name[id]

    def sensor_name2id(self, name):
        if name not in self._sensor_name2id:
            raise ValueError("No \"sensor\" with name %s exists. Available \"sensor\" names = %s." % (name, self.sensor_names))
        return self._sensor_name2id[name]

    cdef inline tuple _extract_mj_names(self, mjModel* p, int*name_adr, int n, mjtObj obj_type):
        cdef char *name
        cdef int obj_id

        # objects don't need to be named in the XML, so name might be None
        id2name = {i: None for i in range(n)}
        name2id = {}
        for i in range(n):
            name = p.names + name_adr[i]
            decoded_name = name.decode()
            if decoded_name:
                obj_id = mj_name2id(p, obj_type, name)
                assert 0 <= obj_id < n and id2name[obj_id] is None
                name2id[decoded_name] = obj_id
                id2name[obj_id] = decoded_name

        # sort names by increasing id to keep order deterministic
        return tuple(id2name[id] for id in sorted(name2id.values())), name2id, id2name

    def get_xml(self):
        cdef char errstr[300]
        cdef int ret
        with TemporaryDirectory() as td:
            filename = os.path.join(td, 'model.xml')
            with wrap_mujoco_warning():
                ret = mj_saveLastXML(filename.encode(), self.ptr, errstr, 300)
            if ret == 0:
                raise Exception('Failed to save XML: {}'.format(errstr))
            return open(filename).read()

    def get_mjb(self):
        with TemporaryDirectory() as td:
            filename = os.path.join(td, 'model.mjb')
            with wrap_mujoco_warning():
                mj_saveModel(self.ptr, filename.encode(), NULL, 0)
            return open(filename, 'rb').read()

    def __dealloc__(self):
        mj_deleteModel(self.ptr)

    def get_joint_qpos_addr(self, name):
        '''
        Returns the qpos address for given joint.

        Returns:
        - address (int, tuple): returns int address if 1-dim joint, otherwise
            returns the a (start, end) tuple for pos[start:end] access.
        '''
        joint_id = self.joint_name2id(name)
        joint_type = self.jnt_type[joint_id]
        joint_addr = self.jnt_qposadr[joint_id]
        if joint_type == mjtJoint.mjJNT_FREE:
            ndim = 7
        elif joint_type == mjtJoint.mjJNT_BALL:
            ndim = 4
        else:
            assert joint_type in (mjtJoint.mjJNT_HINGE, mjtJoint.mjJNT_SLIDE)
            ndim = 1

        if ndim == 1:
            return joint_addr
        else:
            return (joint_addr, joint_addr + ndim)

    def get_joint_qvel_addr(self, name):
        '''
        Returns the qvel address for given joint.

        Returns:
        - address (int, tuple): returns int address if 1-dim joint, otherwise
            returns the a (start, end) tuple for vel[start:end] access.
        '''
        joint_id = self.joint_name2id(name)
        joint_type = self.jnt_type[joint_id]
        joint_addr = self.jnt_dofadr[joint_id]
        if joint_type == mjtJoint.mjJNT_FREE:
            ndim = 6
        elif joint_type == mjtJoint.mjJNT_BALL:
            ndim = 3
        else:
            assert joint_type in (mjtJoint.mjJNT_HINGE, mjtJoint.mjJNT_SLIDE)
            ndim = 1

        if ndim == 1:
            return joint_addr
        else:
            return (joint_addr, joint_addr + ndim)

    cdef void _set(self, mjModel* p):
        
        self.body_names, self._body_name2id, self._body_id2name = self._extract_mj_names(p, p.name_bodyadr, p.nbody, mjtObj.mjOBJ_BODY)
        self.joint_names, self._joint_name2id, self._joint_id2name = self._extract_mj_names(p, p.name_jntadr, p.njnt, mjtObj.mjOBJ_JOINT)
        self.geom_names, self._geom_name2id, self._geom_id2name = self._extract_mj_names(p, p.name_geomadr, p.ngeom, mjtObj.mjOBJ_GEOM)
        self.site_names, self._site_name2id, self._site_id2name = self._extract_mj_names(p, p.name_siteadr, p.nsite, mjtObj.mjOBJ_SITE)
        self.light_names, self._light_name2id, self._light_id2name = self._extract_mj_names(p, p.name_lightadr, p.nlight, mjtObj.mjOBJ_LIGHT)
        self.camera_names, self._camera_name2id, self._camera_id2name = self._extract_mj_names(p, p.name_camadr, p.ncam, mjtObj.mjOBJ_CAMERA)
        self.actuator_names, self._actuator_name2id, self._actuator_id2name = self._extract_mj_names(p, p.name_actuatoradr, p.nu, mjtObj.mjOBJ_ACTUATOR)
        self.sensor_names, self._sensor_name2id, self._sensor_id2name = self._extract_mj_names(p, p.name_sensoradr, p.nsensor, mjtObj.mjOBJ_SENSOR)

        self.ptr = p
        
        
        self._opt = WrapMjOption(&p.opt)
        self._vis = WrapMjVisual(&p.vis)
        self._stat = WrapMjStatistic(&p.stat)
        self._qpos0 = _wrap_mjtNum_1d(p.qpos0, p.nq)
        self._qpos_spring = _wrap_mjtNum_1d(p.qpos_spring, p.nq)
        self._body_parentid = _wrap_int_1d(p.body_parentid, p.nbody)
        self._body_rootid = _wrap_int_1d(p.body_rootid, p.nbody)
        self._body_weldid = _wrap_int_1d(p.body_weldid, p.nbody)
        self._body_mocapid = _wrap_int_1d(p.body_mocapid, p.nbody)
        self._body_jntnum = _wrap_int_1d(p.body_jntnum, p.nbody)
        self._body_jntadr = _wrap_int_1d(p.body_jntadr, p.nbody)
        self._body_dofnum = _wrap_int_1d(p.body_dofnum, p.nbody)
        self._body_dofadr = _wrap_int_1d(p.body_dofadr, p.nbody)
        self._body_geomnum = _wrap_int_1d(p.body_geomnum, p.nbody)
        self._body_geomadr = _wrap_int_1d(p.body_geomadr, p.nbody)
        self._body_pos = _wrap_mjtNum_2d(p.body_pos, p.nbody, 3)
        self._body_quat = _wrap_mjtNum_2d(p.body_quat, p.nbody, 4)
        self._body_ipos = _wrap_mjtNum_2d(p.body_ipos, p.nbody, 3)
        self._body_iquat = _wrap_mjtNum_2d(p.body_iquat, p.nbody, 4)
        self._body_mass = _wrap_mjtNum_1d(p.body_mass, p.nbody)
        self._body_subtreemass = _wrap_mjtNum_1d(p.body_subtreemass, p.nbody)
        self._body_inertia = _wrap_mjtNum_2d(p.body_inertia, p.nbody, 3)
        self._body_invweight0 = _wrap_mjtNum_2d(p.body_invweight0, p.nbody, 2)
        self._body_user = _wrap_mjtNum_2d(p.body_user, p.nbody, p.nuser_body)
        self._jnt_type = _wrap_int_1d(p.jnt_type, p.njnt)
        self._jnt_qposadr = _wrap_int_1d(p.jnt_qposadr, p.njnt)
        self._jnt_dofadr = _wrap_int_1d(p.jnt_dofadr, p.njnt)
        self._jnt_bodyid = _wrap_int_1d(p.jnt_bodyid, p.njnt)
        self._jnt_limited = _wrap_mjtByte_1d(p.jnt_limited, p.njnt)
        self._jnt_solref = _wrap_mjtNum_2d(p.jnt_solref, p.njnt, mjNREF)
        self._jnt_solimp = _wrap_mjtNum_2d(p.jnt_solimp, p.njnt, mjNIMP)
        self._jnt_pos = _wrap_mjtNum_2d(p.jnt_pos, p.njnt, 3)
        self._jnt_axis = _wrap_mjtNum_2d(p.jnt_axis, p.njnt, 3)
        self._jnt_stiffness = _wrap_mjtNum_1d(p.jnt_stiffness, p.njnt)
        self._jnt_range = _wrap_mjtNum_2d(p.jnt_range, p.njnt, 2)
        self._jnt_margin = _wrap_mjtNum_1d(p.jnt_margin, p.njnt)
        self._jnt_user = _wrap_mjtNum_2d(p.jnt_user, p.njnt, p.nuser_jnt)
        self._dof_bodyid = _wrap_int_1d(p.dof_bodyid, p.nv)
        self._dof_jntid = _wrap_int_1d(p.dof_jntid, p.nv)
        self._dof_parentid = _wrap_int_1d(p.dof_parentid, p.nv)
        self._dof_Madr = _wrap_int_1d(p.dof_Madr, p.nv)
        self._dof_solref = _wrap_mjtNum_2d(p.dof_solref, p.nv, mjNREF)
        self._dof_solimp = _wrap_mjtNum_2d(p.dof_solimp, p.nv, mjNIMP)
        self._dof_frictionloss = _wrap_mjtNum_1d(p.dof_frictionloss, p.nv)
        self._dof_armature = _wrap_mjtNum_1d(p.dof_armature, p.nv)
        self._dof_damping = _wrap_mjtNum_1d(p.dof_damping, p.nv)
        self._dof_invweight0 = _wrap_mjtNum_1d(p.dof_invweight0, p.nv)
        self._geom_type = _wrap_int_1d(p.geom_type, p.ngeom)
        self._geom_contype = _wrap_int_1d(p.geom_contype, p.ngeom)
        self._geom_conaffinity = _wrap_int_1d(p.geom_conaffinity, p.ngeom)
        self._geom_condim = _wrap_int_1d(p.geom_condim, p.ngeom)
        self._geom_bodyid = _wrap_int_1d(p.geom_bodyid, p.ngeom)
        self._geom_dataid = _wrap_int_1d(p.geom_dataid, p.ngeom)
        self._geom_matid = _wrap_int_1d(p.geom_matid, p.ngeom)
        self._geom_group = _wrap_int_1d(p.geom_group, p.ngeom)
        self._geom_solmix = _wrap_mjtNum_1d(p.geom_solmix, p.ngeom)
        self._geom_solref = _wrap_mjtNum_2d(p.geom_solref, p.ngeom, mjNREF)
        self._geom_solimp = _wrap_mjtNum_2d(p.geom_solimp, p.ngeom, mjNIMP)
        self._geom_size = _wrap_mjtNum_2d(p.geom_size, p.ngeom, 3)
        self._geom_rbound = _wrap_mjtNum_1d(p.geom_rbound, p.ngeom)
        self._geom_pos = _wrap_mjtNum_2d(p.geom_pos, p.ngeom, 3)
        self._geom_quat = _wrap_mjtNum_2d(p.geom_quat, p.ngeom, 4)
        self._geom_friction = _wrap_mjtNum_2d(p.geom_friction, p.ngeom, 3)
        self._geom_margin = _wrap_mjtNum_1d(p.geom_margin, p.ngeom)
        self._geom_gap = _wrap_mjtNum_1d(p.geom_gap, p.ngeom)
        self._geom_user = _wrap_mjtNum_2d(p.geom_user, p.ngeom, p.nuser_geom)
        self._geom_rgba = _wrap_float_2d(p.geom_rgba, p.ngeom, 4)
        self._site_type = _wrap_int_1d(p.site_type, p.nsite)
        self._site_bodyid = _wrap_int_1d(p.site_bodyid, p.nsite)
        self._site_matid = _wrap_int_1d(p.site_matid, p.nsite)
        self._site_group = _wrap_int_1d(p.site_group, p.nsite)
        self._site_size = _wrap_mjtNum_2d(p.site_size, p.nsite, 3)
        self._site_pos = _wrap_mjtNum_2d(p.site_pos, p.nsite, 3)
        self._site_quat = _wrap_mjtNum_2d(p.site_quat, p.nsite, 4)
        self._site_user = _wrap_mjtNum_2d(p.site_user, p.nsite, p.nuser_site)
        self._site_rgba = _wrap_float_2d(p.site_rgba, p.nsite, 4)
        self._cam_mode = _wrap_int_1d(p.cam_mode, p.ncam)
        self._cam_bodyid = _wrap_int_1d(p.cam_bodyid, p.ncam)
        self._cam_targetbodyid = _wrap_int_1d(p.cam_targetbodyid, p.ncam)
        self._cam_pos = _wrap_mjtNum_2d(p.cam_pos, p.ncam, 3)
        self._cam_quat = _wrap_mjtNum_2d(p.cam_quat, p.ncam, 4)
        self._cam_poscom0 = _wrap_mjtNum_2d(p.cam_poscom0, p.ncam, 3)
        self._cam_pos0 = _wrap_mjtNum_2d(p.cam_pos0, p.ncam, 3)
        self._cam_mat0 = _wrap_mjtNum_2d(p.cam_mat0, p.ncam, 9)
        self._cam_fovy = _wrap_mjtNum_1d(p.cam_fovy, p.ncam)
        self._cam_ipd = _wrap_mjtNum_1d(p.cam_ipd, p.ncam)
        self._cam_user = _wrap_mjtNum_2d(p.cam_user, p.ncam, p.nuser_cam)
        self._light_mode = _wrap_int_1d(p.light_mode, p.nlight)
        self._light_bodyid = _wrap_int_1d(p.light_bodyid, p.nlight)
        self._light_targetbodyid = _wrap_int_1d(p.light_targetbodyid, p.nlight)
        self._light_directional = _wrap_mjtByte_1d(p.light_directional, p.nlight)
        self._light_castshadow = _wrap_mjtByte_1d(p.light_castshadow, p.nlight)
        self._light_active = _wrap_mjtByte_1d(p.light_active, p.nlight)
        self._light_pos = _wrap_mjtNum_2d(p.light_pos, p.nlight, 3)
        self._light_dir = _wrap_mjtNum_2d(p.light_dir, p.nlight, 3)
        self._light_poscom0 = _wrap_mjtNum_2d(p.light_poscom0, p.nlight, 3)
        self._light_pos0 = _wrap_mjtNum_2d(p.light_pos0, p.nlight, 3)
        self._light_dir0 = _wrap_mjtNum_2d(p.light_dir0, p.nlight, 3)
        self._light_attenuation = _wrap_float_2d(p.light_attenuation, p.nlight, 3)
        self._light_cutoff = _wrap_float_1d(p.light_cutoff, p.nlight)
        self._light_exponent = _wrap_float_1d(p.light_exponent, p.nlight)
        self._light_ambient = _wrap_float_2d(p.light_ambient, p.nlight, 3)
        self._light_diffuse = _wrap_float_2d(p.light_diffuse, p.nlight, 3)
        self._light_specular = _wrap_float_2d(p.light_specular, p.nlight, 3)
        self._mesh_faceadr = _wrap_int_1d(p.mesh_faceadr, p.nmesh)
        self._mesh_facenum = _wrap_int_1d(p.mesh_facenum, p.nmesh)
        self._mesh_vertadr = _wrap_int_1d(p.mesh_vertadr, p.nmesh)
        self._mesh_vertnum = _wrap_int_1d(p.mesh_vertnum, p.nmesh)
        self._mesh_graphadr = _wrap_int_1d(p.mesh_graphadr, p.nmesh)
        self._mesh_vert = _wrap_float_2d(p.mesh_vert, p.nmeshvert, 3)
        self._mesh_normal = _wrap_float_2d(p.mesh_normal, p.nmeshvert, 3)
        self._mesh_face = _wrap_int_2d(p.mesh_face, p.nmeshface, 3)
        self._mesh_graph = _wrap_int_1d(p.mesh_graph, p.nmeshgraph)
        self._hfield_size = _wrap_mjtNum_2d(p.hfield_size, p.nhfield, 4)
        self._hfield_nrow = _wrap_int_1d(p.hfield_nrow, p.nhfield)
        self._hfield_ncol = _wrap_int_1d(p.hfield_ncol, p.nhfield)
        self._hfield_adr = _wrap_int_1d(p.hfield_adr, p.nhfield)
        self._hfield_data = _wrap_float_1d(p.hfield_data, p.nhfielddata)
        self._tex_type = _wrap_int_1d(p.tex_type, p.ntex)
        self._tex_height = _wrap_int_1d(p.tex_height, p.ntex)
        self._tex_width = _wrap_int_1d(p.tex_width, p.ntex)
        self._tex_adr = _wrap_int_1d(p.tex_adr, p.ntex)
        self._tex_rgb = _wrap_mjtByte_1d(p.tex_rgb, p.ntexdata)
        self._mat_texid = _wrap_int_1d(p.mat_texid, p.nmat)
        self._mat_texuniform = _wrap_mjtByte_1d(p.mat_texuniform, p.nmat)
        self._mat_texrepeat = _wrap_float_2d(p.mat_texrepeat, p.nmat, 2)
        self._mat_emission = _wrap_float_1d(p.mat_emission, p.nmat)
        self._mat_specular = _wrap_float_1d(p.mat_specular, p.nmat)
        self._mat_shininess = _wrap_float_1d(p.mat_shininess, p.nmat)
        self._mat_reflectance = _wrap_float_1d(p.mat_reflectance, p.nmat)
        self._mat_rgba = _wrap_float_2d(p.mat_rgba, p.nmat, 4)
        self._pair_dim = _wrap_int_1d(p.pair_dim, p.npair)
        self._pair_geom1 = _wrap_int_1d(p.pair_geom1, p.npair)
        self._pair_geom2 = _wrap_int_1d(p.pair_geom2, p.npair)
        self._pair_signature = _wrap_int_1d(p.pair_signature, p.npair)
        self._pair_solref = _wrap_mjtNum_2d(p.pair_solref, p.npair, mjNREF)
        self._pair_solimp = _wrap_mjtNum_2d(p.pair_solimp, p.npair, mjNIMP)
        self._pair_margin = _wrap_mjtNum_1d(p.pair_margin, p.npair)
        self._pair_gap = _wrap_mjtNum_1d(p.pair_gap, p.npair)
        self._pair_friction = _wrap_mjtNum_2d(p.pair_friction, p.npair, 5)
        self._exclude_signature = _wrap_int_1d(p.exclude_signature, p.nexclude)
        self._eq_type = _wrap_int_1d(p.eq_type, p.neq)
        self._eq_obj1id = _wrap_int_1d(p.eq_obj1id, p.neq)
        self._eq_obj2id = _wrap_int_1d(p.eq_obj2id, p.neq)
        self._eq_active = _wrap_mjtByte_1d(p.eq_active, p.neq)
        self._eq_solref = _wrap_mjtNum_2d(p.eq_solref, p.neq, mjNREF)
        self._eq_solimp = _wrap_mjtNum_2d(p.eq_solimp, p.neq, mjNIMP)
        self._eq_data = _wrap_mjtNum_2d(p.eq_data, p.neq, mjNEQDATA)
        self._tendon_adr = _wrap_int_1d(p.tendon_adr, p.ntendon)
        self._tendon_num = _wrap_int_1d(p.tendon_num, p.ntendon)
        self._tendon_matid = _wrap_int_1d(p.tendon_matid, p.ntendon)
        self._tendon_limited = _wrap_mjtByte_1d(p.tendon_limited, p.ntendon)
        self._tendon_width = _wrap_mjtNum_1d(p.tendon_width, p.ntendon)
        self._tendon_solref_lim = _wrap_mjtNum_2d(p.tendon_solref_lim, p.ntendon, mjNREF)
        self._tendon_solimp_lim = _wrap_mjtNum_2d(p.tendon_solimp_lim, p.ntendon, mjNIMP)
        self._tendon_solref_fri = _wrap_mjtNum_2d(p.tendon_solref_fri, p.ntendon, mjNREF)
        self._tendon_solimp_fri = _wrap_mjtNum_2d(p.tendon_solimp_fri, p.ntendon, mjNIMP)
        self._tendon_range = _wrap_mjtNum_2d(p.tendon_range, p.ntendon, 2)
        self._tendon_margin = _wrap_mjtNum_1d(p.tendon_margin, p.ntendon)
        self._tendon_stiffness = _wrap_mjtNum_1d(p.tendon_stiffness, p.ntendon)
        self._tendon_damping = _wrap_mjtNum_1d(p.tendon_damping, p.ntendon)
        self._tendon_frictionloss = _wrap_mjtNum_1d(p.tendon_frictionloss, p.ntendon)
        self._tendon_lengthspring = _wrap_mjtNum_1d(p.tendon_lengthspring, p.ntendon)
        self._tendon_length0 = _wrap_mjtNum_1d(p.tendon_length0, p.ntendon)
        self._tendon_invweight0 = _wrap_mjtNum_1d(p.tendon_invweight0, p.ntendon)
        self._tendon_user = _wrap_mjtNum_2d(p.tendon_user, p.ntendon, p.nuser_tendon)
        self._tendon_rgba = _wrap_float_2d(p.tendon_rgba, p.ntendon, 4)
        self._wrap_type = _wrap_int_1d(p.wrap_type, p.nwrap)
        self._wrap_objid = _wrap_int_1d(p.wrap_objid, p.nwrap)
        self._wrap_prm = _wrap_mjtNum_1d(p.wrap_prm, p.nwrap)
        self._actuator_trntype = _wrap_int_1d(p.actuator_trntype, p.nu)
        self._actuator_dyntype = _wrap_int_1d(p.actuator_dyntype, p.nu)
        self._actuator_gaintype = _wrap_int_1d(p.actuator_gaintype, p.nu)
        self._actuator_biastype = _wrap_int_1d(p.actuator_biastype, p.nu)
        self._actuator_trnid = _wrap_int_2d(p.actuator_trnid, p.nu, 2)
        self._actuator_ctrllimited = _wrap_mjtByte_1d(p.actuator_ctrllimited, p.nu)
        self._actuator_forcelimited = _wrap_mjtByte_1d(p.actuator_forcelimited, p.nu)
        self._actuator_dynprm = _wrap_mjtNum_2d(p.actuator_dynprm, p.nu, mjNDYN)
        self._actuator_gainprm = _wrap_mjtNum_2d(p.actuator_gainprm, p.nu, mjNGAIN)
        self._actuator_biasprm = _wrap_mjtNum_2d(p.actuator_biasprm, p.nu, mjNBIAS)
        self._actuator_ctrlrange = _wrap_mjtNum_2d(p.actuator_ctrlrange, p.nu, 2)
        self._actuator_forcerange = _wrap_mjtNum_2d(p.actuator_forcerange, p.nu, 2)
        self._actuator_gear = _wrap_mjtNum_2d(p.actuator_gear, p.nu, 6)
        self._actuator_cranklength = _wrap_mjtNum_1d(p.actuator_cranklength, p.nu)
        self._actuator_invweight0 = _wrap_mjtNum_1d(p.actuator_invweight0, p.nu)
        self._actuator_length0 = _wrap_mjtNum_1d(p.actuator_length0, p.nu)
        self._actuator_lengthrange = _wrap_mjtNum_2d(p.actuator_lengthrange, p.nu, 2)
        self._actuator_user = _wrap_mjtNum_2d(p.actuator_user, p.nu, p.nuser_actuator)
        self._sensor_type = _wrap_int_1d(p.sensor_type, p.nsensor)
        self._sensor_datatype = _wrap_int_1d(p.sensor_datatype, p.nsensor)
        self._sensor_needstage = _wrap_int_1d(p.sensor_needstage, p.nsensor)
        self._sensor_objtype = _wrap_int_1d(p.sensor_objtype, p.nsensor)
        self._sensor_objid = _wrap_int_1d(p.sensor_objid, p.nsensor)
        self._sensor_dim = _wrap_int_1d(p.sensor_dim, p.nsensor)
        self._sensor_adr = _wrap_int_1d(p.sensor_adr, p.nsensor)
        self._sensor_cutoff = _wrap_mjtNum_1d(p.sensor_cutoff, p.nsensor)
        self._sensor_noise = _wrap_mjtNum_1d(p.sensor_noise, p.nsensor)
        self._sensor_user = _wrap_mjtNum_2d(p.sensor_user, p.nsensor, p.nuser_sensor)
        self._numeric_adr = _wrap_int_1d(p.numeric_adr, p.nnumeric)
        self._numeric_size = _wrap_int_1d(p.numeric_size, p.nnumeric)
        self._numeric_data = _wrap_mjtNum_1d(p.numeric_data, p.nnumericdata)
        self._text_adr = _wrap_int_1d(p.text_adr, p.ntext)
        self._text_size = _wrap_int_1d(p.text_size, p.ntext)
        self._text_data = _wrap_char_1d(p.text_data, p.ntextdata)
        self._tuple_adr = _wrap_int_1d(p.tuple_adr, p.ntuple)
        self._tuple_size = _wrap_int_1d(p.tuple_size, p.ntuple)
        self._tuple_objtype = _wrap_int_1d(p.tuple_objtype, p.ntupledata)
        self._tuple_objid = _wrap_int_1d(p.tuple_objid, p.ntupledata)
        self._tuple_objprm = _wrap_mjtNum_1d(p.tuple_objprm, p.ntupledata)
        self._key_time = _wrap_mjtNum_1d(p.key_time, p.nkey)
        self._key_qpos = _wrap_mjtNum_2d(p.key_qpos, p.nkey, p.nq)
        self._key_qvel = _wrap_mjtNum_2d(p.key_qvel, p.nkey, p.nv)
        self._key_act = _wrap_mjtNum_2d(p.key_act, p.nkey, p.na)
        self._name_bodyadr = _wrap_int_1d(p.name_bodyadr, p.nbody)
        self._name_jntadr = _wrap_int_1d(p.name_jntadr, p.njnt)
        self._name_geomadr = _wrap_int_1d(p.name_geomadr, p.ngeom)
        self._name_siteadr = _wrap_int_1d(p.name_siteadr, p.nsite)
        self._name_camadr = _wrap_int_1d(p.name_camadr, p.ncam)
        self._name_lightadr = _wrap_int_1d(p.name_lightadr, p.nlight)
        self._name_meshadr = _wrap_int_1d(p.name_meshadr, p.nmesh)
        self._name_hfieldadr = _wrap_int_1d(p.name_hfieldadr, p.nhfield)
        self._name_texadr = _wrap_int_1d(p.name_texadr, p.ntex)
        self._name_matadr = _wrap_int_1d(p.name_matadr, p.nmat)
        self._name_eqadr = _wrap_int_1d(p.name_eqadr, p.neq)
        self._name_tendonadr = _wrap_int_1d(p.name_tendonadr, p.ntendon)
        self._name_actuatoradr = _wrap_int_1d(p.name_actuatoradr, p.nu)
        self._name_sensoradr = _wrap_int_1d(p.name_sensoradr, p.nsensor)
        self._name_numericadr = _wrap_int_1d(p.name_numericadr, p.nnumeric)
        self._name_textadr = _wrap_int_1d(p.name_textadr, p.ntext)
        self._name_tupleadr = _wrap_int_1d(p.name_tupleadr, p.ntuple)
        self._names = _wrap_char_1d(p.names, p.nnames)
        
    @property
    def nq(self): return self.ptr.nq
    @nq.setter
    def nq(self, int x): self.ptr.nq = x
    @property
    def nv(self): return self.ptr.nv
    @nv.setter
    def nv(self, int x): self.ptr.nv = x
    @property
    def nu(self): return self.ptr.nu
    @nu.setter
    def nu(self, int x): self.ptr.nu = x
    @property
    def na(self): return self.ptr.na
    @na.setter
    def na(self, int x): self.ptr.na = x
    @property
    def nbody(self): return self.ptr.nbody
    @nbody.setter
    def nbody(self, int x): self.ptr.nbody = x
    @property
    def njnt(self): return self.ptr.njnt
    @njnt.setter
    def njnt(self, int x): self.ptr.njnt = x
    @property
    def ngeom(self): return self.ptr.ngeom
    @ngeom.setter
    def ngeom(self, int x): self.ptr.ngeom = x
    @property
    def nsite(self): return self.ptr.nsite
    @nsite.setter
    def nsite(self, int x): self.ptr.nsite = x
    @property
    def ncam(self): return self.ptr.ncam
    @ncam.setter
    def ncam(self, int x): self.ptr.ncam = x
    @property
    def nlight(self): return self.ptr.nlight
    @nlight.setter
    def nlight(self, int x): self.ptr.nlight = x
    @property
    def nmesh(self): return self.ptr.nmesh
    @nmesh.setter
    def nmesh(self, int x): self.ptr.nmesh = x
    @property
    def nmeshvert(self): return self.ptr.nmeshvert
    @nmeshvert.setter
    def nmeshvert(self, int x): self.ptr.nmeshvert = x
    @property
    def nmeshface(self): return self.ptr.nmeshface
    @nmeshface.setter
    def nmeshface(self, int x): self.ptr.nmeshface = x
    @property
    def nmeshgraph(self): return self.ptr.nmeshgraph
    @nmeshgraph.setter
    def nmeshgraph(self, int x): self.ptr.nmeshgraph = x
    @property
    def nhfield(self): return self.ptr.nhfield
    @nhfield.setter
    def nhfield(self, int x): self.ptr.nhfield = x
    @property
    def nhfielddata(self): return self.ptr.nhfielddata
    @nhfielddata.setter
    def nhfielddata(self, int x): self.ptr.nhfielddata = x
    @property
    def ntex(self): return self.ptr.ntex
    @ntex.setter
    def ntex(self, int x): self.ptr.ntex = x
    @property
    def ntexdata(self): return self.ptr.ntexdata
    @ntexdata.setter
    def ntexdata(self, int x): self.ptr.ntexdata = x
    @property
    def nmat(self): return self.ptr.nmat
    @nmat.setter
    def nmat(self, int x): self.ptr.nmat = x
    @property
    def npair(self): return self.ptr.npair
    @npair.setter
    def npair(self, int x): self.ptr.npair = x
    @property
    def nexclude(self): return self.ptr.nexclude
    @nexclude.setter
    def nexclude(self, int x): self.ptr.nexclude = x
    @property
    def neq(self): return self.ptr.neq
    @neq.setter
    def neq(self, int x): self.ptr.neq = x
    @property
    def ntendon(self): return self.ptr.ntendon
    @ntendon.setter
    def ntendon(self, int x): self.ptr.ntendon = x
    @property
    def nwrap(self): return self.ptr.nwrap
    @nwrap.setter
    def nwrap(self, int x): self.ptr.nwrap = x
    @property
    def nsensor(self): return self.ptr.nsensor
    @nsensor.setter
    def nsensor(self, int x): self.ptr.nsensor = x
    @property
    def nnumeric(self): return self.ptr.nnumeric
    @nnumeric.setter
    def nnumeric(self, int x): self.ptr.nnumeric = x
    @property
    def nnumericdata(self): return self.ptr.nnumericdata
    @nnumericdata.setter
    def nnumericdata(self, int x): self.ptr.nnumericdata = x
    @property
    def ntext(self): return self.ptr.ntext
    @ntext.setter
    def ntext(self, int x): self.ptr.ntext = x
    @property
    def ntextdata(self): return self.ptr.ntextdata
    @ntextdata.setter
    def ntextdata(self, int x): self.ptr.ntextdata = x
    @property
    def ntuple(self): return self.ptr.ntuple
    @ntuple.setter
    def ntuple(self, int x): self.ptr.ntuple = x
    @property
    def ntupledata(self): return self.ptr.ntupledata
    @ntupledata.setter
    def ntupledata(self, int x): self.ptr.ntupledata = x
    @property
    def nkey(self): return self.ptr.nkey
    @nkey.setter
    def nkey(self, int x): self.ptr.nkey = x
    @property
    def nuser_body(self): return self.ptr.nuser_body
    @nuser_body.setter
    def nuser_body(self, int x): self.ptr.nuser_body = x
    @property
    def nuser_jnt(self): return self.ptr.nuser_jnt
    @nuser_jnt.setter
    def nuser_jnt(self, int x): self.ptr.nuser_jnt = x
    @property
    def nuser_geom(self): return self.ptr.nuser_geom
    @nuser_geom.setter
    def nuser_geom(self, int x): self.ptr.nuser_geom = x
    @property
    def nuser_site(self): return self.ptr.nuser_site
    @nuser_site.setter
    def nuser_site(self, int x): self.ptr.nuser_site = x
    @property
    def nuser_cam(self): return self.ptr.nuser_cam
    @nuser_cam.setter
    def nuser_cam(self, int x): self.ptr.nuser_cam = x
    @property
    def nuser_tendon(self): return self.ptr.nuser_tendon
    @nuser_tendon.setter
    def nuser_tendon(self, int x): self.ptr.nuser_tendon = x
    @property
    def nuser_actuator(self): return self.ptr.nuser_actuator
    @nuser_actuator.setter
    def nuser_actuator(self, int x): self.ptr.nuser_actuator = x
    @property
    def nuser_sensor(self): return self.ptr.nuser_sensor
    @nuser_sensor.setter
    def nuser_sensor(self, int x): self.ptr.nuser_sensor = x
    @property
    def nnames(self): return self.ptr.nnames
    @nnames.setter
    def nnames(self, int x): self.ptr.nnames = x
    @property
    def nM(self): return self.ptr.nM
    @nM.setter
    def nM(self, int x): self.ptr.nM = x
    @property
    def nemax(self): return self.ptr.nemax
    @nemax.setter
    def nemax(self, int x): self.ptr.nemax = x
    @property
    def njmax(self): return self.ptr.njmax
    @njmax.setter
    def njmax(self, int x): self.ptr.njmax = x
    @property
    def nconmax(self): return self.ptr.nconmax
    @nconmax.setter
    def nconmax(self, int x): self.ptr.nconmax = x
    @property
    def nstack(self): return self.ptr.nstack
    @nstack.setter
    def nstack(self, int x): self.ptr.nstack = x
    @property
    def nuserdata(self): return self.ptr.nuserdata
    @nuserdata.setter
    def nuserdata(self, int x): self.ptr.nuserdata = x
    @property
    def nmocap(self): return self.ptr.nmocap
    @nmocap.setter
    def nmocap(self, int x): self.ptr.nmocap = x
    @property
    def nsensordata(self): return self.ptr.nsensordata
    @nsensordata.setter
    def nsensordata(self, int x): self.ptr.nsensordata = x
    @property
    def nbuffer(self): return self.ptr.nbuffer
    @nbuffer.setter
    def nbuffer(self, int x): self.ptr.nbuffer = x
    @property
    def opt(self): return self._opt
    @property
    def vis(self): return self._vis
    @property
    def stat(self): return self._stat
    @property
    def qpos0(self): return self._qpos0
    @property
    def qpos_spring(self): return self._qpos_spring
    @property
    def body_parentid(self): return self._body_parentid
    @property
    def body_rootid(self): return self._body_rootid
    @property
    def body_weldid(self): return self._body_weldid
    @property
    def body_mocapid(self): return self._body_mocapid
    @property
    def body_jntnum(self): return self._body_jntnum
    @property
    def body_jntadr(self): return self._body_jntadr
    @property
    def body_dofnum(self): return self._body_dofnum
    @property
    def body_dofadr(self): return self._body_dofadr
    @property
    def body_geomnum(self): return self._body_geomnum
    @property
    def body_geomadr(self): return self._body_geomadr
    @property
    def body_pos(self): return self._body_pos
    @property
    def body_quat(self): return self._body_quat
    @property
    def body_ipos(self): return self._body_ipos
    @property
    def body_iquat(self): return self._body_iquat
    @property
    def body_mass(self): return self._body_mass
    @property
    def body_subtreemass(self): return self._body_subtreemass
    @property
    def body_inertia(self): return self._body_inertia
    @property
    def body_invweight0(self): return self._body_invweight0
    @property
    def body_user(self): return self._body_user
    @property
    def jnt_type(self): return self._jnt_type
    @property
    def jnt_qposadr(self): return self._jnt_qposadr
    @property
    def jnt_dofadr(self): return self._jnt_dofadr
    @property
    def jnt_bodyid(self): return self._jnt_bodyid
    @property
    def jnt_limited(self): return self._jnt_limited
    @property
    def jnt_solref(self): return self._jnt_solref
    @property
    def jnt_solimp(self): return self._jnt_solimp
    @property
    def jnt_pos(self): return self._jnt_pos
    @property
    def jnt_axis(self): return self._jnt_axis
    @property
    def jnt_stiffness(self): return self._jnt_stiffness
    @property
    def jnt_range(self): return self._jnt_range
    @property
    def jnt_margin(self): return self._jnt_margin
    @property
    def jnt_user(self): return self._jnt_user
    @property
    def dof_bodyid(self): return self._dof_bodyid
    @property
    def dof_jntid(self): return self._dof_jntid
    @property
    def dof_parentid(self): return self._dof_parentid
    @property
    def dof_Madr(self): return self._dof_Madr
    @property
    def dof_solref(self): return self._dof_solref
    @property
    def dof_solimp(self): return self._dof_solimp
    @property
    def dof_frictionloss(self): return self._dof_frictionloss
    @property
    def dof_armature(self): return self._dof_armature
    @property
    def dof_damping(self): return self._dof_damping
    @property
    def dof_invweight0(self): return self._dof_invweight0
    @property
    def geom_type(self): return self._geom_type
    @property
    def geom_contype(self): return self._geom_contype
    @property
    def geom_conaffinity(self): return self._geom_conaffinity
    @property
    def geom_condim(self): return self._geom_condim
    @property
    def geom_bodyid(self): return self._geom_bodyid
    @property
    def geom_dataid(self): return self._geom_dataid
    @property
    def geom_matid(self): return self._geom_matid
    @property
    def geom_group(self): return self._geom_group
    @property
    def geom_solmix(self): return self._geom_solmix
    @property
    def geom_solref(self): return self._geom_solref
    @property
    def geom_solimp(self): return self._geom_solimp
    @property
    def geom_size(self): return self._geom_size
    @property
    def geom_rbound(self): return self._geom_rbound
    @property
    def geom_pos(self): return self._geom_pos
    @property
    def geom_quat(self): return self._geom_quat
    @property
    def geom_friction(self): return self._geom_friction
    @property
    def geom_margin(self): return self._geom_margin
    @property
    def geom_gap(self): return self._geom_gap
    @property
    def geom_user(self): return self._geom_user
    @property
    def geom_rgba(self): return self._geom_rgba
    @property
    def site_type(self): return self._site_type
    @property
    def site_bodyid(self): return self._site_bodyid
    @property
    def site_matid(self): return self._site_matid
    @property
    def site_group(self): return self._site_group
    @property
    def site_size(self): return self._site_size
    @property
    def site_pos(self): return self._site_pos
    @property
    def site_quat(self): return self._site_quat
    @property
    def site_user(self): return self._site_user
    @property
    def site_rgba(self): return self._site_rgba
    @property
    def cam_mode(self): return self._cam_mode
    @property
    def cam_bodyid(self): return self._cam_bodyid
    @property
    def cam_targetbodyid(self): return self._cam_targetbodyid
    @property
    def cam_pos(self): return self._cam_pos
    @property
    def cam_quat(self): return self._cam_quat
    @property
    def cam_poscom0(self): return self._cam_poscom0
    @property
    def cam_pos0(self): return self._cam_pos0
    @property
    def cam_mat0(self): return self._cam_mat0
    @property
    def cam_fovy(self): return self._cam_fovy
    @property
    def cam_ipd(self): return self._cam_ipd
    @property
    def cam_user(self): return self._cam_user
    @property
    def light_mode(self): return self._light_mode
    @property
    def light_bodyid(self): return self._light_bodyid
    @property
    def light_targetbodyid(self): return self._light_targetbodyid
    @property
    def light_directional(self): return self._light_directional
    @property
    def light_castshadow(self): return self._light_castshadow
    @property
    def light_active(self): return self._light_active
    @property
    def light_pos(self): return self._light_pos
    @property
    def light_dir(self): return self._light_dir
    @property
    def light_poscom0(self): return self._light_poscom0
    @property
    def light_pos0(self): return self._light_pos0
    @property
    def light_dir0(self): return self._light_dir0
    @property
    def light_attenuation(self): return self._light_attenuation
    @property
    def light_cutoff(self): return self._light_cutoff
    @property
    def light_exponent(self): return self._light_exponent
    @property
    def light_ambient(self): return self._light_ambient
    @property
    def light_diffuse(self): return self._light_diffuse
    @property
    def light_specular(self): return self._light_specular
    @property
    def mesh_faceadr(self): return self._mesh_faceadr
    @property
    def mesh_facenum(self): return self._mesh_facenum
    @property
    def mesh_vertadr(self): return self._mesh_vertadr
    @property
    def mesh_vertnum(self): return self._mesh_vertnum
    @property
    def mesh_graphadr(self): return self._mesh_graphadr
    @property
    def mesh_vert(self): return self._mesh_vert
    @property
    def mesh_normal(self): return self._mesh_normal
    @property
    def mesh_face(self): return self._mesh_face
    @property
    def mesh_graph(self): return self._mesh_graph
    @property
    def hfield_size(self): return self._hfield_size
    @property
    def hfield_nrow(self): return self._hfield_nrow
    @property
    def hfield_ncol(self): return self._hfield_ncol
    @property
    def hfield_adr(self): return self._hfield_adr
    @property
    def hfield_data(self): return self._hfield_data
    @property
    def tex_type(self): return self._tex_type
    @property
    def tex_height(self): return self._tex_height
    @property
    def tex_width(self): return self._tex_width
    @property
    def tex_adr(self): return self._tex_adr
    @property
    def tex_rgb(self): return self._tex_rgb
    @property
    def mat_texid(self): return self._mat_texid
    @property
    def mat_texuniform(self): return self._mat_texuniform
    @property
    def mat_texrepeat(self): return self._mat_texrepeat
    @property
    def mat_emission(self): return self._mat_emission
    @property
    def mat_specular(self): return self._mat_specular
    @property
    def mat_shininess(self): return self._mat_shininess
    @property
    def mat_reflectance(self): return self._mat_reflectance
    @property
    def mat_rgba(self): return self._mat_rgba
    @property
    def pair_dim(self): return self._pair_dim
    @property
    def pair_geom1(self): return self._pair_geom1
    @property
    def pair_geom2(self): return self._pair_geom2
    @property
    def pair_signature(self): return self._pair_signature
    @property
    def pair_solref(self): return self._pair_solref
    @property
    def pair_solimp(self): return self._pair_solimp
    @property
    def pair_margin(self): return self._pair_margin
    @property
    def pair_gap(self): return self._pair_gap
    @property
    def pair_friction(self): return self._pair_friction
    @property
    def exclude_signature(self): return self._exclude_signature
    @property
    def eq_type(self): return self._eq_type
    @property
    def eq_obj1id(self): return self._eq_obj1id
    @property
    def eq_obj2id(self): return self._eq_obj2id
    @property
    def eq_active(self): return self._eq_active
    @property
    def eq_solref(self): return self._eq_solref
    @property
    def eq_solimp(self): return self._eq_solimp
    @property
    def eq_data(self): return self._eq_data
    @property
    def tendon_adr(self): return self._tendon_adr
    @property
    def tendon_num(self): return self._tendon_num
    @property
    def tendon_matid(self): return self._tendon_matid
    @property
    def tendon_limited(self): return self._tendon_limited
    @property
    def tendon_width(self): return self._tendon_width
    @property
    def tendon_solref_lim(self): return self._tendon_solref_lim
    @property
    def tendon_solimp_lim(self): return self._tendon_solimp_lim
    @property
    def tendon_solref_fri(self): return self._tendon_solref_fri
    @property
    def tendon_solimp_fri(self): return self._tendon_solimp_fri
    @property
    def tendon_range(self): return self._tendon_range
    @property
    def tendon_margin(self): return self._tendon_margin
    @property
    def tendon_stiffness(self): return self._tendon_stiffness
    @property
    def tendon_damping(self): return self._tendon_damping
    @property
    def tendon_frictionloss(self): return self._tendon_frictionloss
    @property
    def tendon_lengthspring(self): return self._tendon_lengthspring
    @property
    def tendon_length0(self): return self._tendon_length0
    @property
    def tendon_invweight0(self): return self._tendon_invweight0
    @property
    def tendon_user(self): return self._tendon_user
    @property
    def tendon_rgba(self): return self._tendon_rgba
    @property
    def wrap_type(self): return self._wrap_type
    @property
    def wrap_objid(self): return self._wrap_objid
    @property
    def wrap_prm(self): return self._wrap_prm
    @property
    def actuator_trntype(self): return self._actuator_trntype
    @property
    def actuator_dyntype(self): return self._actuator_dyntype
    @property
    def actuator_gaintype(self): return self._actuator_gaintype
    @property
    def actuator_biastype(self): return self._actuator_biastype
    @property
    def actuator_trnid(self): return self._actuator_trnid
    @property
    def actuator_ctrllimited(self): return self._actuator_ctrllimited
    @property
    def actuator_forcelimited(self): return self._actuator_forcelimited
    @property
    def actuator_dynprm(self): return self._actuator_dynprm
    @property
    def actuator_gainprm(self): return self._actuator_gainprm
    @property
    def actuator_biasprm(self): return self._actuator_biasprm
    @property
    def actuator_ctrlrange(self): return self._actuator_ctrlrange
    @property
    def actuator_forcerange(self): return self._actuator_forcerange
    @property
    def actuator_gear(self): return self._actuator_gear
    @property
    def actuator_cranklength(self): return self._actuator_cranklength
    @property
    def actuator_invweight0(self): return self._actuator_invweight0
    @property
    def actuator_length0(self): return self._actuator_length0
    @property
    def actuator_lengthrange(self): return self._actuator_lengthrange
    @property
    def actuator_user(self): return self._actuator_user
    @property
    def sensor_type(self): return self._sensor_type
    @property
    def sensor_datatype(self): return self._sensor_datatype
    @property
    def sensor_needstage(self): return self._sensor_needstage
    @property
    def sensor_objtype(self): return self._sensor_objtype
    @property
    def sensor_objid(self): return self._sensor_objid
    @property
    def sensor_dim(self): return self._sensor_dim
    @property
    def sensor_adr(self): return self._sensor_adr
    @property
    def sensor_cutoff(self): return self._sensor_cutoff
    @property
    def sensor_noise(self): return self._sensor_noise
    @property
    def sensor_user(self): return self._sensor_user
    @property
    def numeric_adr(self): return self._numeric_adr
    @property
    def numeric_size(self): return self._numeric_size
    @property
    def numeric_data(self): return self._numeric_data
    @property
    def text_adr(self): return self._text_adr
    @property
    def text_size(self): return self._text_size
    @property
    def text_data(self): return self._text_data
    @property
    def tuple_adr(self): return self._tuple_adr
    @property
    def tuple_size(self): return self._tuple_size
    @property
    def tuple_objtype(self): return self._tuple_objtype
    @property
    def tuple_objid(self): return self._tuple_objid
    @property
    def tuple_objprm(self): return self._tuple_objprm
    @property
    def key_time(self): return self._key_time
    @property
    def key_qpos(self): return self._key_qpos
    @property
    def key_qvel(self): return self._key_qvel
    @property
    def key_act(self): return self._key_act
    @property
    def name_bodyadr(self): return self._name_bodyadr
    @property
    def name_jntadr(self): return self._name_jntadr
    @property
    def name_geomadr(self): return self._name_geomadr
    @property
    def name_siteadr(self): return self._name_siteadr
    @property
    def name_camadr(self): return self._name_camadr
    @property
    def name_lightadr(self): return self._name_lightadr
    @property
    def name_meshadr(self): return self._name_meshadr
    @property
    def name_hfieldadr(self): return self._name_hfieldadr
    @property
    def name_texadr(self): return self._name_texadr
    @property
    def name_matadr(self): return self._name_matadr
    @property
    def name_eqadr(self): return self._name_eqadr
    @property
    def name_tendonadr(self): return self._name_tendonadr
    @property
    def name_actuatoradr(self): return self._name_actuatoradr
    @property
    def name_sensoradr(self): return self._name_sensoradr
    @property
    def name_numericadr(self): return self._name_numericadr
    @property
    def name_textadr(self): return self._name_textadr
    @property
    def name_tupleadr(self): return self._name_tupleadr
    @property
    def names(self): return self._names

cdef PyMjModel WrapMjModel(mjModel* p):
    cdef PyMjModel o = PyMjModel()
    o._set(p)
    return o

cdef inline np.ndarray _wrap_char_1d(char* a, int shape0):
    if shape0 == 0: return None
    cdef char[:] b = <char[:shape0]> a
    return np.asarray(b)

cdef inline np.ndarray _wrap_float_1d(float* a, int shape0):
    if shape0 == 0: return None
    cdef float[:] b = <float[:shape0]> a
    return np.asarray(b)

cdef inline np.ndarray _wrap_int_1d(int* a, int shape0):
    if shape0 == 0: return None
    cdef int[:] b = <int[:shape0]> a
    return np.asarray(b)

cdef inline np.ndarray _wrap_mjtByte_1d(mjtByte* a, int shape0):
    if shape0 == 0: return None
    cdef mjtByte[:] b = <mjtByte[:shape0]> a
    return np.asarray(b)

cdef inline np.ndarray _wrap_mjtNum_1d(mjtNum* a, int shape0):
    if shape0 == 0: return None
    cdef mjtNum[:] b = <mjtNum[:shape0]> a
    return np.asarray(b)

cdef inline np.ndarray _wrap_unsigned_int_1d(unsigned int* a, int shape0):
    if shape0 == 0: return None
    cdef unsigned int[:] b = <unsigned int[:shape0]> a
    return np.asarray(b)

cdef inline np.ndarray _wrap_char_2d(char* a, int shape0, int shape1):
    if shape0 * shape1 == 0: return None
    cdef char[:,:] b = <char[:shape0,:shape1]> a
    return np.asarray(b)

cdef inline np.ndarray _wrap_float_2d(float* a, int shape0, int shape1):
    if shape0 * shape1 == 0: return None
    cdef float[:,:] b = <float[:shape0,:shape1]> a
    return np.asarray(b)

cdef inline np.ndarray _wrap_int_2d(int* a, int shape0, int shape1):
    if shape0 * shape1 == 0: return None
    cdef int[:,:] b = <int[:shape0,:shape1]> a
    return np.asarray(b)

cdef inline np.ndarray _wrap_mjtByte_2d(mjtByte* a, int shape0, int shape1):
    if shape0 * shape1 == 0: return None
    cdef mjtByte[:,:] b = <mjtByte[:shape0,:shape1]> a
    return np.asarray(b)

cdef inline np.ndarray _wrap_mjtNum_2d(mjtNum* a, int shape0, int shape1):
    if shape0 * shape1 == 0: return None
    cdef mjtNum[:,:] b = <mjtNum[:shape0,:shape1]> a
    return np.asarray(b)

def _mj_activate(str filename):
    return mj_activate(filename.encode())

def _mj_deactivate():
    mj_deactivate()

def _mj_certQuestion(np.ndarray[np.float64_t, mode="c", ndim=1] question):
    mj_certQuestion(&question[0])

def _mj_certAnswer(np.ndarray[np.float64_t, mode="c", ndim=1] question, np.ndarray[np.float64_t, mode="c", ndim=1] answer):
    mj_certAnswer(&question[0], &answer[0])

def _mj_certCheck(np.ndarray[np.float64_t, mode="c", ndim=1] question, np.ndarray[np.float64_t, mode="c", ndim=1] answer):
    return mj_certCheck(&question[0], &answer[0])

def _mj_defaultVFS(PyMjVFS vfs):
    mj_defaultVFS(vfs.ptr)

def _mj_addFileVFS(PyMjVFS vfs, str directory, str filename):
    return mj_addFileVFS(vfs.ptr, directory.encode(), filename.encode())

def _mj_makeEmptyFileVFS(PyMjVFS vfs, str filename, int filesize):
    return mj_makeEmptyFileVFS(vfs.ptr, filename.encode(), filesize)

def _mj_findFileVFS(PyMjVFS vfs, str filename):
    return mj_findFileVFS(vfs.ptr, filename.encode())

def _mj_deleteFileVFS(PyMjVFS vfs, str filename):
    return mj_deleteFileVFS(vfs.ptr, filename.encode())

def _mj_deleteVFS(PyMjVFS vfs):
    mj_deleteVFS(vfs.ptr)

def _mj_loadXML(str filename, PyMjVFS vfs, str error, int error_sz):
    return WrapMjModel(mj_loadXML(filename.encode(), vfs.ptr, error.encode(), error_sz))

def _mj_saveLastXML(str filename, PyMjModel m, str error, int error_sz):
    return mj_saveLastXML(filename.encode(), m.ptr, error.encode(), error_sz)

def _mj_freeLastXML():
    mj_freeLastXML()

def _mj_printSchema(str filename, str buffer, int buffer_sz, int flg_html, int flg_pad):
    return mj_printSchema(filename.encode(), buffer.encode(), buffer_sz, flg_html, flg_pad)

def _mj_step(PyMjModel m, PyMjData d):
    mj_step(m.ptr, d.ptr)

def _mj_step1(PyMjModel m, PyMjData d):
    mj_step1(m.ptr, d.ptr)

def _mj_step2(PyMjModel m, PyMjData d):
    mj_step2(m.ptr, d.ptr)

def _mj_forward(PyMjModel m, PyMjData d):
    mj_forward(m.ptr, d.ptr)

def _mj_inverse(PyMjModel m, PyMjData d):
    mj_inverse(m.ptr, d.ptr)

def _mj_forwardSkip(PyMjModel m, PyMjData d, int skipstage, int skipsensorenergy):
    mj_forwardSkip(m.ptr, d.ptr, skipstage, skipsensorenergy)

def _mj_inverseSkip(PyMjModel m, PyMjData d, int skipstage, int skipsensorenergy):
    mj_inverseSkip(m.ptr, d.ptr, skipstage, skipsensorenergy)

def _mj_defaultSolRefImp(np.ndarray[np.float64_t, mode="c", ndim=1] solref, np.ndarray[np.float64_t, mode="c", ndim=1] solimp):
    mj_defaultSolRefImp(&solref[0], &solimp[0])

def _mj_defaultOption(PyMjOption opt):
    mj_defaultOption(opt.ptr)

def _mj_defaultVisual(PyMjVisual vis):
    mj_defaultVisual(vis.ptr)

def _mj_copyModel(PyMjModel dest, PyMjModel src):
    return WrapMjModel(mj_copyModel(dest.ptr, src.ptr))

def _mj_loadModel(str filename, PyMjVFS vfs):
    return WrapMjModel(mj_loadModel(filename.encode(), vfs.ptr))

def _mj_deleteModel(PyMjModel m):
    mj_deleteModel(m.ptr)

def _mj_sizeModel(PyMjModel m):
    return mj_sizeModel(m.ptr)

def _mj_resetData(PyMjModel m, PyMjData d):
    mj_resetData(m.ptr, d.ptr)

def _mj_resetDataKeyframe(PyMjModel m, PyMjData d, int key):
    mj_resetDataKeyframe(m.ptr, d.ptr, key)

def _mj_deleteData(PyMjData d):
    mj_deleteData(d.ptr)

def _mj_resetCallbacks():
    mj_resetCallbacks()

def _mj_setConst(PyMjModel m, PyMjData d, int flg_actrange):
    mj_setConst(m.ptr, d.ptr, flg_actrange)

def _mj_printModel(PyMjModel m, str filename):
    mj_printModel(m.ptr, filename.encode())

def _mj_printData(PyMjModel m, PyMjData d, str filename):
    mj_printData(m.ptr, d.ptr, filename.encode())

def _mju_printMat(np.ndarray[np.float64_t, mode="c", ndim=1] mat, int nr, int nc):
    mju_printMat(&mat[0], nr, nc)

def _mj_fwdPosition(PyMjModel m, PyMjData d):
    mj_fwdPosition(m.ptr, d.ptr)

def _mj_fwdVelocity(PyMjModel m, PyMjData d):
    mj_fwdVelocity(m.ptr, d.ptr)

def _mj_fwdActuation(PyMjModel m, PyMjData d):
    mj_fwdActuation(m.ptr, d.ptr)

def _mj_fwdAcceleration(PyMjModel m, PyMjData d):
    mj_fwdAcceleration(m.ptr, d.ptr)

def _mj_fwdConstraint(PyMjModel m, PyMjData d):
    mj_fwdConstraint(m.ptr, d.ptr)

def _mj_Euler(PyMjModel m, PyMjData d):
    mj_Euler(m.ptr, d.ptr)

def _mj_RungeKutta(PyMjModel m, PyMjData d, int N):
    mj_RungeKutta(m.ptr, d.ptr, N)

def _mj_invPosition(PyMjModel m, PyMjData d):
    mj_invPosition(m.ptr, d.ptr)

def _mj_invVelocity(PyMjModel m, PyMjData d):
    mj_invVelocity(m.ptr, d.ptr)

def _mj_invConstraint(PyMjModel m, PyMjData d):
    mj_invConstraint(m.ptr, d.ptr)

def _mj_compareFwdInv(PyMjModel m, PyMjData d):
    mj_compareFwdInv(m.ptr, d.ptr)

def _mj_sensorPos(PyMjModel m, PyMjData d):
    mj_sensorPos(m.ptr, d.ptr)

def _mj_sensorVel(PyMjModel m, PyMjData d):
    mj_sensorVel(m.ptr, d.ptr)

def _mj_sensorAcc(PyMjModel m, PyMjData d):
    mj_sensorAcc(m.ptr, d.ptr)

def _mj_energyPos(PyMjModel m, PyMjData d):
    mj_energyPos(m.ptr, d.ptr)

def _mj_energyVel(PyMjModel m, PyMjData d):
    mj_energyVel(m.ptr, d.ptr)

def _mj_checkPos(PyMjModel m, PyMjData d):
    mj_checkPos(m.ptr, d.ptr)

def _mj_checkVel(PyMjModel m, PyMjData d):
    mj_checkVel(m.ptr, d.ptr)

def _mj_checkAcc(PyMjModel m, PyMjData d):
    mj_checkAcc(m.ptr, d.ptr)

def _mj_kinematics(PyMjModel m, PyMjData d):
    mj_kinematics(m.ptr, d.ptr)

def _mj_comPos(PyMjModel m, PyMjData d):
    mj_comPos(m.ptr, d.ptr)

def _mj_camlight(PyMjModel m, PyMjData d):
    mj_camlight(m.ptr, d.ptr)

def _mj_tendon(PyMjModel m, PyMjData d):
    mj_tendon(m.ptr, d.ptr)

def _mj_transmission(PyMjModel m, PyMjData d):
    mj_transmission(m.ptr, d.ptr)

def _mj_crb(PyMjModel m, PyMjData d):
    mj_crb(m.ptr, d.ptr)

def _mj_factorM(PyMjModel m, PyMjData d):
    mj_factorM(m.ptr, d.ptr)

def _mj_solveM(PyMjModel m, PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] x, np.ndarray[np.float64_t, mode="c", ndim=1] y, int n):
    mj_solveM(m.ptr, d.ptr, &x[0], &y[0], n)

def _mj_solveM2(PyMjModel m, PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] x, np.ndarray[np.float64_t, mode="c", ndim=1] y, int n):
    mj_solveM2(m.ptr, d.ptr, &x[0], &y[0], n)

def _mj_comVel(PyMjModel m, PyMjData d):
    mj_comVel(m.ptr, d.ptr)

def _mj_passive(PyMjModel m, PyMjData d):
    mj_passive(m.ptr, d.ptr)

def _mj_rne(PyMjModel m, PyMjData d, int flg_acc, np.ndarray[np.float64_t, mode="c", ndim=1] result):
    mj_rne(m.ptr, d.ptr, flg_acc, &result[0])

def _mj_rnePostConstraint(PyMjModel m, PyMjData d):
    mj_rnePostConstraint(m.ptr, d.ptr)

def _mj_collision(PyMjModel m, PyMjData d):
    mj_collision(m.ptr, d.ptr)

def _mj_makeConstraint(PyMjModel m, PyMjData d):
    mj_makeConstraint(m.ptr, d.ptr)

def _mj_projectConstraint(PyMjModel m, PyMjData d):
    mj_projectConstraint(m.ptr, d.ptr)

def _mj_referenceConstraint(PyMjModel m, PyMjData d):
    mj_referenceConstraint(m.ptr, d.ptr)

def _mj_constraintUpdate(PyMjModel m, PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] jar, np.ndarray[np.float64_t, mode="c", ndim=1] cost, int flg_coneHessian):
    mj_constraintUpdate(m.ptr, d.ptr, &jar[0], &cost[0], flg_coneHessian)

def _mj_addContact(PyMjModel m, PyMjData d, PyMjContact con):
    return mj_addContact(m.ptr, d.ptr, con.ptr)

def _mj_isPyramidal(PyMjModel m):
    return mj_isPyramidal(m.ptr)

def _mj_isSparse(PyMjModel m):
    return mj_isSparse(m.ptr)

def _mj_isDual(PyMjModel m):
    return mj_isDual(m.ptr)

def _mj_mulJacVec(PyMjModel m, PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec):
    mj_mulJacVec(m.ptr, d.ptr, &res[0], &vec[0])

def _mj_mulJacTVec(PyMjModel m, PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec):
    mj_mulJacTVec(m.ptr, d.ptr, &res[0], &vec[0])

def _mj_jac(PyMjModel m, PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] jacp, np.ndarray[np.float64_t, mode="c", ndim=1] jacr, np.ndarray[np.float64_t, mode="c", ndim=1] point, int body):
    mj_jac(m.ptr, d.ptr, &jacp[0], &jacr[0], &point[0], body)

def _mj_jacBody(PyMjModel m, PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] jacp, np.ndarray[np.float64_t, mode="c", ndim=1] jacr, int body):
    mj_jacBody(m.ptr, d.ptr, &jacp[0], &jacr[0], body)

def _mj_jacBodyCom(PyMjModel m, PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] jacp, np.ndarray[np.float64_t, mode="c", ndim=1] jacr, int body):
    mj_jacBodyCom(m.ptr, d.ptr, &jacp[0], &jacr[0], body)

def _mj_jacGeom(PyMjModel m, PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] jacp, np.ndarray[np.float64_t, mode="c", ndim=1] jacr, int geom):
    mj_jacGeom(m.ptr, d.ptr, &jacp[0], &jacr[0], geom)

def _mj_jacSite(PyMjModel m, PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] jacp, np.ndarray[np.float64_t, mode="c", ndim=1] jacr, int site):
    mj_jacSite(m.ptr, d.ptr, &jacp[0], &jacr[0], site)

def _mj_jacPointAxis(PyMjModel m, PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] jacPoint, np.ndarray[np.float64_t, mode="c", ndim=1] jacAxis, np.ndarray[np.float64_t, mode="c", ndim=1] point, np.ndarray[np.float64_t, mode="c", ndim=1] axis, int body):
    mj_jacPointAxis(m.ptr, d.ptr, &jacPoint[0], &jacAxis[0], &point[0], &axis[0], body)

def _mj_name2id(PyMjModel m, int type, str name):
    return mj_name2id(m.ptr, type, name.encode())

def _mj_fullM(PyMjModel m, np.ndarray[np.float64_t, mode="c", ndim=1] dst, np.ndarray[np.float64_t, mode="c", ndim=1] M):
    mj_fullM(m.ptr, &dst[0], &M[0])

def _mj_mulM(PyMjModel m, PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec):
    mj_mulM(m.ptr, d.ptr, &res[0], &vec[0])

def _mj_applyFT(PyMjModel m, PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] force, np.ndarray[np.float64_t, mode="c", ndim=1] torque, np.ndarray[np.float64_t, mode="c", ndim=1] point, int body, np.ndarray[np.float64_t, mode="c", ndim=1] qfrc_target):
    mj_applyFT(m.ptr, d.ptr, &force[0], &torque[0], &point[0], body, &qfrc_target[0])

def _mj_objectVelocity(PyMjModel m, PyMjData d, int objtype, int objid, np.ndarray[np.float64_t, mode="c", ndim=1] res, int flg_local):
    mj_objectVelocity(m.ptr, d.ptr, objtype, objid, &res[0], flg_local)

def _mj_objectAcceleration(PyMjModel m, PyMjData d, int objtype, int objid, np.ndarray[np.float64_t, mode="c", ndim=1] res, int flg_local):
    mj_objectAcceleration(m.ptr, d.ptr, objtype, objid, &res[0], flg_local)

def _mj_differentiatePos(PyMjModel m, np.ndarray[np.float64_t, mode="c", ndim=1] qvel, float dt, np.ndarray[np.float64_t, mode="c", ndim=1] qpos1, np.ndarray[np.float64_t, mode="c", ndim=1] qpos2):
    mj_differentiatePos(m.ptr, &qvel[0], dt, &qpos1[0], &qpos2[0])

def _mj_contactForce(PyMjModel m, PyMjData d, int id, np.ndarray[np.float64_t, mode="c", ndim=1] result):
    mj_contactForce(m.ptr, d.ptr, id, &result[0])

def _mj_integratePos(PyMjModel m, np.ndarray[np.float64_t, mode="c", ndim=1] qpos, np.ndarray[np.float64_t, mode="c", ndim=1] qvel, float dt):
    mj_integratePos(m.ptr, &qpos[0], &qvel[0], dt)

def _mj_normalizeQuat(PyMjModel m, np.ndarray[np.float64_t, mode="c", ndim=1] qpos):
    mj_normalizeQuat(m.ptr, &qpos[0])

def _mj_local2Global(PyMjData d, np.ndarray[np.float64_t, mode="c", ndim=1] xpos, np.ndarray[np.float64_t, mode="c", ndim=1] xmat, np.ndarray[np.float64_t, mode="c", ndim=1] pos, np.ndarray[np.float64_t, mode="c", ndim=1] quat, int body):
    mj_local2Global(d.ptr, &xpos[0], &xmat[0], &pos[0], &quat[0], body)

def _mj_getTotalmass(PyMjModel m):
    return mj_getTotalmass(m.ptr)

def _mj_setTotalmass(PyMjModel m, float newmass):
    mj_setTotalmass(m.ptr, newmass)

def _mj_version():
    return mj_version()

def _mj_rayHfield(PyMjModel m, PyMjData d, int geomid, np.ndarray[np.float64_t, mode="c", ndim=1] pnt, np.ndarray[np.float64_t, mode="c", ndim=1] vec):
    return mj_rayHfield(m.ptr, d.ptr, geomid, &pnt[0], &vec[0])

def _mj_rayMesh(PyMjModel m, PyMjData d, int geomid, np.ndarray[np.float64_t, mode="c", ndim=1] pnt, np.ndarray[np.float64_t, mode="c", ndim=1] vec):
    return mj_rayMesh(m.ptr, d.ptr, geomid, &pnt[0], &vec[0])

def _mju_rayGeom(np.ndarray[np.float64_t, mode="c", ndim=1] pos, np.ndarray[np.float64_t, mode="c", ndim=1] mat, np.ndarray[np.float64_t, mode="c", ndim=1] size, np.ndarray[np.float64_t, mode="c", ndim=1] pnt, np.ndarray[np.float64_t, mode="c", ndim=1] vec, int geomtype):
    return mju_rayGeom(&pos[0], &mat[0], &size[0], &pnt[0], &vec[0], geomtype)

def _mjv_defaultCamera(PyMjvCamera cam):
    mjv_defaultCamera(cam.ptr)

def _mjv_defaultPerturb(PyMjvPerturb pert):
    mjv_defaultPerturb(pert.ptr)

def _mjv_room2model(np.ndarray[np.float64_t, mode="c", ndim=1] modelpos, np.ndarray[np.float64_t, mode="c", ndim=1] modelquat, np.ndarray[np.float64_t, mode="c", ndim=1] roompos, np.ndarray[np.float64_t, mode="c", ndim=1] roomquat, PyMjvScene scn):
    mjv_room2model(&modelpos[0], &modelquat[0], &roompos[0], &roomquat[0], scn.ptr)

def _mjv_model2room(np.ndarray[np.float64_t, mode="c", ndim=1] roompos, np.ndarray[np.float64_t, mode="c", ndim=1] roomquat, np.ndarray[np.float64_t, mode="c", ndim=1] modelpos, np.ndarray[np.float64_t, mode="c", ndim=1] modelquat, PyMjvScene scn):
    mjv_model2room(&roompos[0], &roomquat[0], &modelpos[0], &modelquat[0], scn.ptr)

def _mjv_cameraInModel(np.ndarray[np.float64_t, mode="c", ndim=1] headpos, np.ndarray[np.float64_t, mode="c", ndim=1] forward, np.ndarray[np.float64_t, mode="c", ndim=1] up, PyMjvScene scn):
    mjv_cameraInModel(&headpos[0], &forward[0], &up[0], scn.ptr)

def _mjv_cameraInRoom(np.ndarray[np.float64_t, mode="c", ndim=1] headpos, np.ndarray[np.float64_t, mode="c", ndim=1] forward, np.ndarray[np.float64_t, mode="c", ndim=1] up, PyMjvScene scn):
    mjv_cameraInRoom(&headpos[0], &forward[0], &up[0], scn.ptr)

def _mjv_frustumHeight(PyMjvScene scn):
    return mjv_frustumHeight(scn.ptr)

def _mjv_alignToCamera(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec, np.ndarray[np.float64_t, mode="c", ndim=1] forward):
    mjv_alignToCamera(&res[0], &vec[0], &forward[0])

def _mjv_moveCamera(PyMjModel m, int action, float reldx, float reldy, PyMjvScene scn, PyMjvCamera cam):
    mjv_moveCamera(m.ptr, action, reldx, reldy, scn.ptr, cam.ptr)

def _mjv_movePerturb(PyMjModel m, PyMjData d, int action, float reldx, float reldy, PyMjvScene scn, PyMjvPerturb pert):
    mjv_movePerturb(m.ptr, d.ptr, action, reldx, reldy, scn.ptr, pert.ptr)

def _mjv_moveModel(PyMjModel m, int action, float reldx, float reldy, np.ndarray[np.float64_t, mode="c", ndim=1] roomup, PyMjvScene scn):
    mjv_moveModel(m.ptr, action, reldx, reldy, &roomup[0], scn.ptr)

def _mjv_initPerturb(PyMjModel m, PyMjData d, PyMjvScene scn, PyMjvPerturb pert):
    mjv_initPerturb(m.ptr, d.ptr, scn.ptr, pert.ptr)

def _mjv_applyPerturbPose(PyMjModel m, PyMjData d, PyMjvPerturb pert, int flg_paused):
    mjv_applyPerturbPose(m.ptr, d.ptr, pert.ptr, flg_paused)

def _mjv_applyPerturbForce(PyMjModel m, PyMjData d, PyMjvPerturb pert):
    mjv_applyPerturbForce(m.ptr, d.ptr, pert.ptr)

def _mjv_select(PyMjModel m, PyMjData d, PyMjvOption vopt, float aspectratio, float relx, float rely, PyMjvScene scn, np.ndarray[np.float64_t, mode="c", ndim=1] selpnt):
    return mjv_select(m.ptr, d.ptr, vopt.ptr, aspectratio, relx, rely, scn.ptr, &selpnt[0])

def _mjv_defaultOption(PyMjvOption opt):
    mjv_defaultOption(opt.ptr)

def _mjv_defaultFigure(PyMjvFigure fig):
    mjv_defaultFigure(fig.ptr)

def _mjv_makeConnector(PyMjvGeom geom, int type, float width, float a0, float a1, float a2, float b0, float b1, float b2):
    mjv_makeConnector(geom.ptr, type, width, a0, a1, a2, b0, b1, b2)

def _mjv_makeScene(PyMjvScene scn, int maxgeom):
    mjv_makeScene(scn.ptr, maxgeom)

def _mjv_freeScene(PyMjvScene scn):
    mjv_freeScene(scn.ptr)

def _mjv_updateScene(PyMjModel m, PyMjData d, PyMjvOption opt, PyMjvPerturb pert, PyMjvCamera cam, int catmask, PyMjvScene scn):
    mjv_updateScene(m.ptr, d.ptr, opt.ptr, pert.ptr, cam.ptr, catmask, scn.ptr)

def _mjv_addGeoms(PyMjModel m, PyMjData d, PyMjvOption opt, PyMjvPerturb pert, int catmask, PyMjvScene scn):
    mjv_addGeoms(m.ptr, d.ptr, opt.ptr, pert.ptr, catmask, scn.ptr)

def _mjv_updateCamera(PyMjModel m, PyMjData d, PyMjvCamera cam, PyMjvScene scn):
    mjv_updateCamera(m.ptr, d.ptr, cam.ptr, scn.ptr)

def _mjr_defaultContext(PyMjrContext con):
    mjr_defaultContext(con.ptr)

def _mjr_makeContext(PyMjModel m, PyMjrContext con, int fontscale):
    mjr_makeContext(m.ptr, con.ptr, fontscale)

def _mjr_freeContext(PyMjrContext con):
    mjr_freeContext(con.ptr)

def _mjr_uploadTexture(PyMjModel m, PyMjrContext con, int texid):
    mjr_uploadTexture(m.ptr, con.ptr, texid)

def _mjr_uploadMesh(PyMjModel m, PyMjrContext con, int meshid):
    mjr_uploadMesh(m.ptr, con.ptr, meshid)

def _mjr_uploadHField(PyMjModel m, PyMjrContext con, int hfieldid):
    mjr_uploadHField(m.ptr, con.ptr, hfieldid)

def _mjr_setBuffer(int framebuffer, PyMjrContext con):
    mjr_setBuffer(framebuffer, con.ptr)

def _mjr_blitBuffer(PyMjrRect src, PyMjrRect dst, int flg_color, int flg_depth, PyMjrContext con):
    mjr_blitBuffer(src.ptr[0], dst.ptr[0], flg_color, flg_depth, con.ptr)

def _mjr_overlay(int font, int gridpos, PyMjrRect viewport, str overlay, str overlay2, PyMjrContext con):
    mjr_overlay(font, gridpos, viewport.ptr[0], overlay.encode(), overlay2.encode(), con.ptr)

def _mjr_figure(PyMjrRect viewport, PyMjvFigure fig, PyMjrContext con):
    mjr_figure(viewport.ptr[0], fig.ptr, con.ptr)

def _mjr_render(PyMjrRect viewport, PyMjvScene scn, PyMjrContext con):
    mjr_render(viewport.ptr[0], scn.ptr, con.ptr)

def _mjr_finish():
    mjr_finish()

def _mjr_getError():
    return mjr_getError()

def _mju_error(str msg):
    mju_error(msg.encode())

def _mju_error_i(str msg, int i):
    mju_error_i(msg.encode(), i)

def _mju_error_s(str msg, str text):
    mju_error_s(msg.encode(), text.encode())

def _mju_warning(str msg):
    mju_warning(msg.encode())

def _mju_warning_i(str msg, int i):
    mju_warning_i(msg.encode(), i)

def _mju_warning_s(str msg, str text):
    mju_warning_s(msg.encode(), text.encode())

def _mju_clearHandlers():
    mju_clearHandlers()

def _mj_warning(PyMjData d, int warning, int info):
    mj_warning(d.ptr, warning, info)

def _mju_writeLog(str type, str msg):
    mju_writeLog(type.encode(), msg.encode())

def _mju_zero3(np.ndarray[np.float64_t, mode="c", ndim=1] res):
    mju_zero3(&res[0])

def _mju_copy3(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] data):
    mju_copy3(&res[0], &data[0])

def _mju_scl3(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec, float scl):
    mju_scl3(&res[0], &vec[0], scl)

def _mju_add3(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec1, np.ndarray[np.float64_t, mode="c", ndim=1] vec2):
    mju_add3(&res[0], &vec1[0], &vec2[0])

def _mju_sub3(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec1, np.ndarray[np.float64_t, mode="c", ndim=1] vec2):
    mju_sub3(&res[0], &vec1[0], &vec2[0])

def _mju_addTo3(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec):
    mju_addTo3(&res[0], &vec[0])

def _mju_subFrom3(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec):
    mju_subFrom3(&res[0], &vec[0])

def _mju_addToScl3(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec, float scl):
    mju_addToScl3(&res[0], &vec[0], scl)

def _mju_addScl3(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec1, np.ndarray[np.float64_t, mode="c", ndim=1] vec2, float scl):
    mju_addScl3(&res[0], &vec1[0], &vec2[0], scl)

def _mju_normalize3(np.ndarray[np.float64_t, mode="c", ndim=1] res):
    return mju_normalize3(&res[0])

def _mju_norm3(np.ndarray[np.float64_t, mode="c", ndim=1] vec):
    return mju_norm3(&vec[0])

def _mju_dot3(np.ndarray[np.float64_t, mode="c", ndim=1] vec1, np.ndarray[np.float64_t, mode="c", ndim=1] vec2):
    return mju_dot3(&vec1[0], &vec2[0])

def _mju_dist3(np.ndarray[np.float64_t, mode="c", ndim=1] pos1, np.ndarray[np.float64_t, mode="c", ndim=1] pos2):
    return mju_dist3(&pos1[0], &pos2[0])

def _mju_rotVecMat(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec, np.ndarray[np.float64_t, mode="c", ndim=1] mat):
    mju_rotVecMat(&res[0], &vec[0], &mat[0])

def _mju_rotVecMatT(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec, np.ndarray[np.float64_t, mode="c", ndim=1] mat):
    mju_rotVecMatT(&res[0], &vec[0], &mat[0])

def _mju_cross(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] a, np.ndarray[np.float64_t, mode="c", ndim=1] b):
    mju_cross(&res[0], &a[0], &b[0])

def _mju_zero4(np.ndarray[np.float64_t, mode="c", ndim=1] res):
    mju_zero4(&res[0])

def _mju_unit4(np.ndarray[np.float64_t, mode="c", ndim=1] res):
    mju_unit4(&res[0])

def _mju_copy4(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] data):
    mju_copy4(&res[0], &data[0])

def _mju_normalize4(np.ndarray[np.float64_t, mode="c", ndim=1] res):
    return mju_normalize4(&res[0])

def _mju_zero(np.ndarray[np.float64_t, mode="c", ndim=1] res, int n):
    mju_zero(&res[0], n)

def _mju_copy(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] data, int n):
    mju_copy(&res[0], &data[0], n)

def _mju_scl(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec, float scl, int n):
    mju_scl(&res[0], &vec[0], scl, n)

def _mju_add(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec1, np.ndarray[np.float64_t, mode="c", ndim=1] vec2, int n):
    mju_add(&res[0], &vec1[0], &vec2[0], n)

def _mju_sub(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec1, np.ndarray[np.float64_t, mode="c", ndim=1] vec2, int n):
    mju_sub(&res[0], &vec1[0], &vec2[0], n)

def _mju_addTo(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec, int n):
    mju_addTo(&res[0], &vec[0], n)

def _mju_subFrom(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec, int n):
    mju_subFrom(&res[0], &vec[0], n)

def _mju_addToScl(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec, float scl, int n):
    mju_addToScl(&res[0], &vec[0], scl, n)

def _mju_addScl(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec1, np.ndarray[np.float64_t, mode="c", ndim=1] vec2, float scl, int n):
    mju_addScl(&res[0], &vec1[0], &vec2[0], scl, n)

def _mju_normalize(np.ndarray[np.float64_t, mode="c", ndim=1] res, int n):
    return mju_normalize(&res[0], n)

def _mju_norm(np.ndarray[np.float64_t, mode="c", ndim=1] res, int n):
    return mju_norm(&res[0], n)

def _mju_dot(np.ndarray[np.float64_t, mode="c", ndim=1] vec1, np.ndarray[np.float64_t, mode="c", ndim=1] vec2, int n):
    return mju_dot(&vec1[0], &vec2[0], n)

def _mju_mulMatVec(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] mat, np.ndarray[np.float64_t, mode="c", ndim=1] vec, int nr, int nc):
    mju_mulMatVec(&res[0], &mat[0], &vec[0], nr, nc)

def _mju_mulMatTVec(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] mat, np.ndarray[np.float64_t, mode="c", ndim=1] vec, int nr, int nc):
    mju_mulMatTVec(&res[0], &mat[0], &vec[0], nr, nc)

def _mju_transpose(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] mat, int nr, int nc):
    mju_transpose(&res[0], &mat[0], nr, nc)

def _mju_mulMatMat(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] mat1, np.ndarray[np.float64_t, mode="c", ndim=1] mat2, int r1, int c1, int c2):
    mju_mulMatMat(&res[0], &mat1[0], &mat2[0], r1, c1, c2)

def _mju_mulMatMatT(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] mat1, np.ndarray[np.float64_t, mode="c", ndim=1] mat2, int r1, int c1, int r2):
    mju_mulMatMatT(&res[0], &mat1[0], &mat2[0], r1, c1, r2)

def _mju_mulMatTMat(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] mat1, np.ndarray[np.float64_t, mode="c", ndim=1] mat2, int r1, int c1, int c2):
    mju_mulMatTMat(&res[0], &mat1[0], &mat2[0], r1, c1, c2)

def _mju_sqrMatTD(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] mat, np.ndarray[np.float64_t, mode="c", ndim=1] diag, int nr, int nc):
    mju_sqrMatTD(&res[0], &mat[0], &diag[0], nr, nc)

def _mju_transformSpatial(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec, int flg_force, np.ndarray[np.float64_t, mode="c", ndim=1] newpos, np.ndarray[np.float64_t, mode="c", ndim=1] oldpos, np.ndarray[np.float64_t, mode="c", ndim=1] rotnew2old):
    mju_transformSpatial(&res[0], &vec[0], flg_force, &newpos[0], &oldpos[0], &rotnew2old[0])

def _mju_rotVecQuat(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] vec, np.ndarray[np.float64_t, mode="c", ndim=1] quat):
    mju_rotVecQuat(&res[0], &vec[0], &quat[0])

def _mju_negQuat(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] quat):
    mju_negQuat(&res[0], &quat[0])

def _mju_mulQuat(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] quat1, np.ndarray[np.float64_t, mode="c", ndim=1] quat2):
    mju_mulQuat(&res[0], &quat1[0], &quat2[0])

def _mju_mulQuatAxis(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] quat, np.ndarray[np.float64_t, mode="c", ndim=1] axis):
    mju_mulQuatAxis(&res[0], &quat[0], &axis[0])

def _mju_axisAngle2Quat(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] axis, float angle):
    mju_axisAngle2Quat(&res[0], &axis[0], angle)

def _mju_quat2Vel(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] quat, float dt):
    mju_quat2Vel(&res[0], &quat[0], dt)

def _mju_quat2Mat(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] quat):
    mju_quat2Mat(&res[0], &quat[0])

def _mju_mat2Quat(np.ndarray[np.float64_t, mode="c", ndim=1] quat, np.ndarray[np.float64_t, mode="c", ndim=1] mat):
    mju_mat2Quat(&quat[0], &mat[0])

def _mju_derivQuat(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] quat, np.ndarray[np.float64_t, mode="c", ndim=1] vel):
    mju_derivQuat(&res[0], &quat[0], &vel[0])

def _mju_quatIntegrate(np.ndarray[np.float64_t, mode="c", ndim=1] quat, np.ndarray[np.float64_t, mode="c", ndim=1] vel, float scale):
    mju_quatIntegrate(&quat[0], &vel[0], scale)

def _mju_quatZ2Vec(np.ndarray[np.float64_t, mode="c", ndim=1] quat, np.ndarray[np.float64_t, mode="c", ndim=1] vec):
    mju_quatZ2Vec(&quat[0], &vec[0])

def _mju_mulPose(np.ndarray[np.float64_t, mode="c", ndim=1] posres, np.ndarray[np.float64_t, mode="c", ndim=1] quatres, np.ndarray[np.float64_t, mode="c", ndim=1] pos1, np.ndarray[np.float64_t, mode="c", ndim=1] quat1, np.ndarray[np.float64_t, mode="c", ndim=1] pos2, np.ndarray[np.float64_t, mode="c", ndim=1] quat2):
    mju_mulPose(&posres[0], &quatres[0], &pos1[0], &quat1[0], &pos2[0], &quat2[0])

def _mju_negPose(np.ndarray[np.float64_t, mode="c", ndim=1] posres, np.ndarray[np.float64_t, mode="c", ndim=1] quatres, np.ndarray[np.float64_t, mode="c", ndim=1] pos, np.ndarray[np.float64_t, mode="c", ndim=1] quat):
    mju_negPose(&posres[0], &quatres[0], &pos[0], &quat[0])

def _mju_trnVecPose(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] pos, np.ndarray[np.float64_t, mode="c", ndim=1] quat, np.ndarray[np.float64_t, mode="c", ndim=1] vec):
    mju_trnVecPose(&res[0], &pos[0], &quat[0], &vec[0])

def _mju_cholFactor(np.ndarray[np.float64_t, mode="c", ndim=1] mat, int n):
    return mju_cholFactor(&mat[0], n)

def _mju_cholSolve(np.ndarray[np.float64_t, mode="c", ndim=1] res, np.ndarray[np.float64_t, mode="c", ndim=1] mat, np.ndarray[np.float64_t, mode="c", ndim=1] vec, int n):
    mju_cholSolve(&res[0], &mat[0], &vec[0], n)

def _mju_cholUpdate(np.ndarray[np.float64_t, mode="c", ndim=1] mat, np.ndarray[np.float64_t, mode="c", ndim=1] x, int n, int flg_plus):
    return mju_cholUpdate(&mat[0], &x[0], n, flg_plus)

def _mju_eig3(np.ndarray[np.float64_t, mode="c", ndim=1] eigval, np.ndarray[np.float64_t, mode="c", ndim=1] eigvec, np.ndarray[np.float64_t, mode="c", ndim=1] quat, np.ndarray[np.float64_t, mode="c", ndim=1] mat):
    return mju_eig3(&eigval[0], &eigvec[0], &quat[0], &mat[0])

def _mju_muscleFVL(float len, float vel, float lmin, float lmax, np.ndarray[np.float64_t, mode="c", ndim=1] prm):
    return mju_muscleFVL(len, vel, lmin, lmax, &prm[0])

def _mju_musclePassive(float len, float lmin, float lmax, np.ndarray[np.float64_t, mode="c", ndim=1] prm):
    return mju_musclePassive(len, lmin, lmax, &prm[0])

def _mju_pneumatic(float len, float len0, float vel, np.ndarray[np.float64_t, mode="c", ndim=1] prm, float act, float ctrl, float timestep, np.ndarray[np.float64_t, mode="c", ndim=1] jac):
    return mju_pneumatic(len, len0, vel, &prm[0], act, ctrl, timestep, &jac[0])

def _mju_encodePyramid(np.ndarray[np.float64_t, mode="c", ndim=1] pyramid, np.ndarray[np.float64_t, mode="c", ndim=1] force, np.ndarray[np.float64_t, mode="c", ndim=1] mu, int dim):
    mju_encodePyramid(&pyramid[0], &force[0], &mu[0], dim)

def _mju_decodePyramid(np.ndarray[np.float64_t, mode="c", ndim=1] force, np.ndarray[np.float64_t, mode="c", ndim=1] pyramid, np.ndarray[np.float64_t, mode="c", ndim=1] mu, int dim):
    mju_decodePyramid(&force[0], &pyramid[0], &mu[0], dim)

def _mju_springDamper(float pos0, float vel0, float Kp, float Kv, float dt):
    return mju_springDamper(pos0, vel0, Kp, Kv, dt)

def _mju_min(float a, float b):
    return mju_min(a, b)

def _mju_max(float a, float b):
    return mju_max(a, b)

def _mju_sign(float x):
    return mju_sign(x)

def _mju_round(float x):
    return mju_round(x)

def _mju_str2Type(str str):
    return mju_str2Type(str.encode())

def _mju_isBad(float x):
    return mju_isBad(x)

def _mju_isZero(np.ndarray[np.float64_t, mode="c", ndim=1] vec, int n):
    return mju_isZero(&vec[0], n)

def _mju_standardNormal(np.ndarray[np.float64_t, mode="c", ndim=1] num2):
    return mju_standardNormal(&num2[0])

def _mju_insertionSort(np.ndarray[np.float64_t, mode="c", ndim=1] list, int n):
    mju_insertionSort(&list[0], n)

def _mju_Halton(int index, int base):
    return mju_Halton(index, base)


