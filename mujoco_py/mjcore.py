from ctypes import create_string_buffer
import ctypes
from . import mjconstants as C

from .mjtypes import *  # import all for backwards compatibility
from .mjlib import mjlib
from six.moves import xrange

def register_license(file_path):
    """
    activates mujoco with license at `file_path`

    this does not check the return code, per usage example at simulate.cpp
    and test.cpp.
    """
    result = mjlib.mj_activate(file_path)
    return result


class dict2(dict):
    def __init__(self, **kwargs):
        dict.__init__(self, kwargs)
        self.__dict__ = self


class MjModel(MjModelWrapper):

    def __init__(self, xml_path):
        buf = create_string_buffer(1000)
        model_ptr = mjlib.mj_loadXML(xml_path, None, buf, 1000)
        if len(buf.value) > 0:
            print("Warning: %s" % buf.value)
        super(MjModel, self).__init__(model_ptr)
        data_ptr = mjlib.mj_makeData(model_ptr)
        fields = ["nq","nv","na","nu","nbody","nmocap","nuserdata","nsensordata","njnt","ngeom","nsite","ncam","nlight","ntendon","nwrap","nM","njmax","nemax"]
        sizes = dict2(**{ k: getattr(self, k) for k in fields })
        data = MjData(data_ptr, sizes)
        self.data = data
        self._body_comvels = None
        self.forward()

    def forward(self):
        mjlib.mj_forward(self.ptr, self.data.ptr)
        mjlib.mj_sensor(self.ptr, self.data.ptr)
        mjlib.mj_energy(self.ptr, self.data.ptr)
        self._body_comvels = None

    @property
    def body_comvels(self):
        if self._body_comvels is None:
            self._body_comvels = self._compute_subtree()
        return self._body_comvels

    def _compute_subtree(self):
        body_vels = np.zeros((self.nbody, 6))
        # bodywise quantities
        mass = self.body_mass.flatten()
        for i in xrange(self.nbody):
            # body velocity
            mjlib.mj_objectVelocity(
                self.ptr, self.data.ptr, C.mjOBJ_BODY, i,
                body_vels[i].ctypes.data_as(POINTER(c_double)), 0
            )
            # body linear momentum
        lin_moms = body_vels[:, 3:] * mass.reshape((-1, 1))

        # init subtree mass
        body_parentid = self.body_parentid
        # subtree com and com_vel
        for i in xrange(self.nbody - 1, -1, -1):
            if i > 0:
                parent = body_parentid[i]
                # add scaled velocities
                lin_moms[parent] += lin_moms[i]
                # accumulate mass
                mass[parent] += mass[i]
        return lin_moms / mass.reshape((-1, 1))

    def step(self):
        mjlib.mj_step(self.ptr, self.data.ptr)

    def __del__(self):
        if self._wrapped is not None:
            # At the very end of the process, mjlib can be unloaded before we are deleted.
            # At that point, it's okay to leak this memory.
            if mjlib: mjlib.mj_deleteModel(self._wrapped)

    @property
    def body_names(self):
        start_addr = ctypes.addressof(self.names.contents)
        return [ctypes.string_at(start_addr + int(inc))
                for inc in self.name_bodyadr.flatten()]

    @property
    def joint_names(self):
        start_addr = ctypes.addressof(self.names.contents)
        return [ctypes.string_at(start_addr + int(inc))
                for inc in self.name_jntadr.flatten()]

    def joint_adr(self, joint_name):
        """Return (qposadr, qveladr, dof) for the given joint name.

        If dof is 4 or 7, then the last 4 degrees of freedom in qpos represent a
        unit quaternion."""
        jntadr = mjlib.mj_name2id(self.ptr, C.mjOBJ_JOINT, joint_name)
        assert(jntadr >= 0)
        dofmap = {C.mjJNT_FREE:  7,
                  C.mjJNT_BALL:  4,
                  C.mjJNT_SLIDE: 1,
                  C.mjJNT_HINGE: 1}
        qposadr = self.jnt_qposadr[jntadr][0]
        qveladr = self.jnt_dofadr[jntadr][0]
        dof     = dofmap[self.jnt_type[jntadr][0]]
        return (qposadr, qveladr, dof)

    @property
    def geom_names(self):
        start_addr = ctypes.addressof(self.names.contents)
        return [ctypes.string_at(start_addr + int(inc))
                for inc in self.name_geomadr.flatten()]

    @property
    def site_names(self):
        start_addr = ctypes.addressof(self.names.contents)
        return [ctypes.string_at(start_addr + int(inc))
                for inc in self.name_siteadr.flatten()]

    @property
    def mesh_names(self):
        start_addr = ctypes.addressof(self.names.contents)
        return [ctypes.string_at(start_addr + int(inc))
                for inc in self.name_meshadr.flatten()]

    @property
    def numeric_names(self):
        start_addr = ctypes.addressof(self.names.contents)
        return [ctypes.string_at(start_addr + int(inc))
                for inc in self.name_numericadr.flatten()]

    @property
    def actuator_names(self):
        start_addr = ctypes.addressof(self.names.contents)
        return [ctypes.string_at(start_addr + int(inc))
                for inc in self.name_actuatoradr.flatten()]

    @property
    def camera_names(self):
        start_addr = ctypes.addressof(self.names.contents)
        return [ctypes.string_at(start_addr + int(inc))
                for inc in self.name_camadr.flatten()]


class MjData(MjDataWrapper):

    def __init__(self, wrapped, size_src=None):
        super(MjData, self).__init__(wrapped, size_src)

    def __del__(self):
        if self._wrapped is not None:
            # At the very end of the process, mjlib can be unloaded before we are deleted.
            # At that point, it's okay to leak this memory.
            if mjlib: mjlib.mj_deleteData(self._wrapped)
