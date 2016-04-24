
# AUTO GENERATED. DO NOT CHANGE!
from ctypes import *
import numpy as np

class MJCONTACT(Structure):
    
    _fields_ = [
        ("dist", c_double),
        ("pos", c_double * 3),
        ("frame", c_double * 9),
        ("includemargin", c_double),
        ("friction", c_double * 5),
        ("solref", c_double * 2),
        ("solimp", c_double * 3),
        ("mu", c_double),
        ("coef", c_double * 5),
        ("zone", c_int),
        ("dim", c_int),
        ("geom1", c_int),
        ("geom2", c_int),
        ("exclude", c_int),
        ("efc_address", c_int),
    ]

class MJRRECT(Structure):
    
    _fields_ = [
        ("left", c_int),
        ("bottom", c_int),
        ("width", c_int),
        ("height", c_int),
    ]

class MJVCAMERAPOSE(Structure):
    
    _fields_ = [
        ("head_pos", c_double * 3),
        ("head_right", c_double * 3),
        ("window_pos", c_double * 3),
        ("window_right", c_double * 3),
        ("window_up", c_double * 3),
        ("window_normal", c_double * 3),
        ("window_size", c_double * 2),
        ("scale", c_double),
        ("ipd", c_double),
    ]

class MJROPTION(Structure):
    
    _fields_ = [
        ("stereo", c_ubyte),
        ("flags", c_ubyte * 6),
    ]

class MJRCONTEXT(Structure):
    
    _fields_ = [
        ("linewidth", c_float),
        ("znear", c_float),
        ("zfar", c_float),
        ("shadowclip", c_float),
        ("shadowscale", c_float),
        ("shadowsize", c_int),
        ("offwidth", c_uint),
        ("offheight", c_uint),
        ("offFBO", c_uint),
        ("offColor", c_uint),
        ("offDepthStencil", c_uint),
        ("shadowFBO", c_uint),
        ("shadowTex", c_uint),
        ("ntexture", c_uint),
        ("texture", c_int * 100),
        ("textureType", c_int * 100),
        ("basePlane", c_uint),
        ("baseMesh", c_uint),
        ("baseHField", c_uint),
        ("baseBuiltin", c_uint),
        ("baseFontNormal", c_uint),
        ("baseFontBack", c_uint),
        ("baseFontBig", c_uint),
        ("rangePlane", c_int),
        ("rangeMesh", c_int),
        ("rangeHField", c_int),
        ("rangeBuiltin", c_int),
        ("rangeFont", c_int),
        ("charWidth", c_int * 127),
        ("charWidthBig", c_int * 127),
        ("charHeight", c_int),
        ("charHeightBig", c_int),
        ("glewInitialized", c_int),
    ]

class MJVCAMERA(Structure):
    
    _fields_ = [
        ("fovy", c_double),
        ("camid", c_int),
        ("trackbodyid", c_int),
        ("lookat", c_double * 3),
        ("azimuth", c_double),
        ("elevation", c_double),
        ("distance", c_double),
        ("pose", MJVCAMERAPOSE),
        ("VR", c_ubyte),
    ]

class MJVOPTION(Structure):
    
    _fields_ = [
        ("label", c_int),
        ("frame", c_int),
        ("geomgroup", c_ubyte * 5),
        ("sitegroup", c_ubyte * 5),
        ("flags", c_ubyte * 18),
    ]

class MJVGEOM(Structure):
    
    _fields_ = [
        ("type", c_int),
        ("dataid", c_int),
        ("objtype", c_int),
        ("objid", c_int),
        ("category", c_int),
        ("texid", c_int),
        ("texuniform", c_int),
        ("texrepeat", c_float * 2),
        ("size", c_float * 3),
        ("pos", c_float * 3),
        ("mat", c_float * 9),
        ("rgba", c_float * 4),
        ("emission", c_float),
        ("specular", c_float),
        ("shininess", c_float),
        ("reflectance", c_float),
        ("label", c_char * 100),
        ("camdist", c_float),
        ("rbound", c_float),
        ("transparent", c_ubyte),
    ]

class MJVLIGHT(Structure):
    
    _fields_ = [
        ("pos", c_float * 3),
        ("dir", c_float * 3),
        ("attenuation", c_float * 3),
        ("cutoff", c_float),
        ("exponent", c_float),
        ("ambient", c_float * 3),
        ("diffuse", c_float * 3),
        ("specular", c_float * 3),
        ("headlight", c_ubyte),
        ("directional", c_ubyte),
        ("castshadow", c_ubyte),
    ]

class MJVOBJECTS(Structure):
    
    _fields_ = [
        ("nlight", c_int),
        ("ngeom", c_int),
        ("maxgeom", c_int),
        ("lights", MJVLIGHT * 8),
        ("geoms", POINTER(MJVGEOM)),
        ("geomorder", POINTER(c_int)),
    ]

class MJOPTION(Structure):
    
    _fields_ = [
        ("timestep", c_double),
        ("apirate", c_double),
        ("tolerance", c_double),
        ("impratio", c_double),
        ("gravity", c_double * 3),
        ("wind", c_double * 3),
        ("magnetic", c_double * 3),
        ("density", c_double),
        ("viscosity", c_double),
        ("o_margin", c_double),
        ("o_solref", c_double * 2),
        ("o_solimp", c_double * 3),
        ("mpr_tolerance", c_double),
        ("mpr_iterations", c_int),
        ("integrator", c_int),
        ("collision", c_int),
        ("impedance", c_int),
        ("reference", c_int),
        ("solver", c_int),
        ("iterations", c_int),
        ("disableflags", c_int),
        ("enableflags", c_int),
    ]

class MJVISUAL(Structure):
    
    class ANON_GLOBAL(Structure):
        
        _fields_ = [
            ("fovy", c_float),
            ("ipd", c_float),
            ("linewidth", c_float),
            ("glow", c_float),
            ("offwidth", c_int),
            ("offheight", c_int),
        ]
    
    class ANON_QUALITY(Structure):
        
        _fields_ = [
            ("shadowsize", c_int),
            ("numSlices", c_int),
            ("numStacks", c_int),
            ("numArrows", c_int),
            ("numQuads", c_int),
        ]
    
    class ANON_HEADLIGHT(Structure):
        
        _fields_ = [
            ("ambient", c_float * 3),
            ("diffuse", c_float * 3),
            ("specular", c_float * 3),
            ("active", c_int),
        ]
    
    class ANON_MAP(Structure):
        
        _fields_ = [
            ("stiffness", c_float),
            ("force", c_float),
            ("torque", c_float),
            ("alpha", c_float),
            ("fogstart", c_float),
            ("fogend", c_float),
            ("znear", c_float),
            ("zfar", c_float),
            ("shadowclip", c_float),
            ("shadowscale", c_float),
        ]
    
    class ANON_SCALE(Structure):
        
        _fields_ = [
            ("forcewidth", c_float),
            ("contactwidth", c_float),
            ("contactheight", c_float),
            ("connect", c_float),
            ("com", c_float),
            ("camera", c_float),
            ("light", c_float),
            ("selectpoint", c_float),
            ("jointlength", c_float),
            ("jointwidth", c_float),
            ("actuatorlength", c_float),
            ("actuatorwidth", c_float),
            ("framelength", c_float),
            ("framewidth", c_float),
            ("constraint", c_float),
            ("slidercrank", c_float),
        ]
    
    class ANON_RGBA(Structure):
        
        _fields_ = [
            ("fog", c_float * 4),
            ("force", c_float * 4),
            ("inertia", c_float * 4),
            ("joint", c_float * 4),
            ("actuator", c_float * 4),
            ("com", c_float * 4),
            ("camera", c_float * 4),
            ("light", c_float * 4),
            ("selectpoint", c_float * 4),
            ("connect", c_float * 4),
            ("contactpoint", c_float * 4),
            ("contactforce", c_float * 4),
            ("contactfriction", c_float * 4),
            ("contacttorque", c_float * 4),
            ("constraint", c_float * 4),
            ("slidercrank", c_float * 4),
            ("crankbroken", c_float * 4),
        ]
    _fields_ = [
        ("global_", ANON_GLOBAL),
        ("quality", ANON_QUALITY),
        ("headlight", ANON_HEADLIGHT),
        ("map_", ANON_MAP),
        ("scale", ANON_SCALE),
        ("rgba", ANON_RGBA),
    ]

class MJSTATISTIC(Structure):
    
    _fields_ = [
        ("meanmass", c_double),
        ("meansize", c_double),
        ("extent", c_double),
        ("center", c_double * 3),
    ]

class MJDATA(Structure):
    
    _fields_ = [
        ("nstack", c_int),
        ("nbuffer", c_int),
        ("pstack", c_int),
        ("maxstackuse", c_int),
        ("ne", c_int),
        ("nf", c_int),
        ("nefc", c_int),
        ("ncon", c_int),
        ("nwarning", c_int * 8),
        ("warning_info", c_int * 8),
        ("timer_duration", c_double * 14),
        ("timer_ncall", c_double * 14),
        ("mocaptime", c_double * 3),
        ("time", c_double),
        ("energy", c_double * 2),
        ("solverstat", c_double * 4),
        ("solvertrace", c_double * 200),
        ("buffer", POINTER(c_ubyte)),
        ("stack", POINTER(c_double)),
        ("qpos", POINTER(c_double)),
        ("qvel", POINTER(c_double)),
        ("act", POINTER(c_double)),
        ("ctrl", POINTER(c_double)),
        ("qfrc_applied", POINTER(c_double)),
        ("xfrc_applied", POINTER(c_double)),
        ("qacc", POINTER(c_double)),
        ("act_dot", POINTER(c_double)),
        ("mocap_pos", POINTER(c_double)),
        ("mocap_quat", POINTER(c_double)),
        ("userdata", POINTER(c_double)),
        ("sensordata", POINTER(c_double)),
        ("xpos", POINTER(c_double)),
        ("xquat", POINTER(c_double)),
        ("xmat", POINTER(c_double)),
        ("xipos", POINTER(c_double)),
        ("ximat", POINTER(c_double)),
        ("xanchor", POINTER(c_double)),
        ("xaxis", POINTER(c_double)),
        ("geom_xpos", POINTER(c_double)),
        ("geom_xmat", POINTER(c_double)),
        ("site_xpos", POINTER(c_double)),
        ("site_xmat", POINTER(c_double)),
        ("cam_xpos", POINTER(c_double)),
        ("cam_xmat", POINTER(c_double)),
        ("light_xpos", POINTER(c_double)),
        ("light_xdir", POINTER(c_double)),
        ("com_subtree", POINTER(c_double)),
        ("cdof", POINTER(c_double)),
        ("cinert", POINTER(c_double)),
        ("ten_wrapadr", POINTER(c_int)),
        ("ten_wrapnum", POINTER(c_int)),
        ("ten_length", POINTER(c_double)),
        ("ten_moment", POINTER(c_double)),
        ("wrap_obj", POINTER(c_int)),
        ("wrap_xpos", POINTER(c_double)),
        ("actuator_length", POINTER(c_double)),
        ("actuator_moment", POINTER(c_double)),
        ("crb", POINTER(c_double)),
        ("qM", POINTER(c_double)),
        ("qLD", POINTER(c_double)),
        ("qLDiagInv", POINTER(c_double)),
        ("qLDiagSqrtInv", POINTER(c_double)),
        ("contact", POINTER(MJCONTACT)),
        ("efc_type", POINTER(c_int)),
        ("efc_id", POINTER(c_int)),
        ("efc_rownnz", POINTER(c_int)),
        ("efc_rowadr", POINTER(c_int)),
        ("efc_colind", POINTER(c_int)),
        ("efc_rownnz_T", POINTER(c_int)),
        ("efc_rowadr_T", POINTER(c_int)),
        ("efc_colind_T", POINTER(c_int)),
        ("efc_solref", POINTER(c_double)),
        ("efc_solimp", POINTER(c_double)),
        ("efc_margin", POINTER(c_double)),
        ("efc_frictionloss", POINTER(c_double)),
        ("efc_pos", POINTER(c_double)),
        ("efc_J", POINTER(c_double)),
        ("efc_J_T", POINTER(c_double)),
        ("efc_diagApprox", POINTER(c_double)),
        ("efc_D", POINTER(c_double)),
        ("efc_R", POINTER(c_double)),
        ("efc_AR", POINTER(c_double)),
        ("e_ARchol", POINTER(c_double)),
        ("fc_e_rect", POINTER(c_double)),
        ("fc_AR", POINTER(c_double)),
        ("ten_velocity", POINTER(c_double)),
        ("actuator_velocity", POINTER(c_double)),
        ("cvel", POINTER(c_double)),
        ("cdof_dot", POINTER(c_double)),
        ("qfrc_bias", POINTER(c_double)),
        ("qfrc_passive", POINTER(c_double)),
        ("efc_vel", POINTER(c_double)),
        ("efc_aref", POINTER(c_double)),
        ("actuator_force", POINTER(c_double)),
        ("qfrc_actuator", POINTER(c_double)),
        ("qfrc_unc", POINTER(c_double)),
        ("qacc_unc", POINTER(c_double)),
        ("efc_b", POINTER(c_double)),
        ("fc_b", POINTER(c_double)),
        ("efc_force", POINTER(c_double)),
        ("qfrc_constraint", POINTER(c_double)),
        ("qfrc_inverse", POINTER(c_double)),
        ("cacc", POINTER(c_double)),
        ("cfrc_int", POINTER(c_double)),
        ("cfrc_ext", POINTER(c_double)),
    ]

class MJMODEL(Structure):
    
    _fields_ = [
        ("nq", c_int),
        ("nv", c_int),
        ("nu", c_int),
        ("na", c_int),
        ("nbody", c_int),
        ("njnt", c_int),
        ("ngeom", c_int),
        ("nsite", c_int),
        ("ncam", c_int),
        ("nlight", c_int),
        ("nmesh", c_int),
        ("nmeshvert", c_int),
        ("nmeshface", c_int),
        ("nmeshgraph", c_int),
        ("nhfield", c_int),
        ("nhfielddata", c_int),
        ("ntex", c_int),
        ("ntexdata", c_int),
        ("nmat", c_int),
        ("npair", c_int),
        ("nexclude", c_int),
        ("neq", c_int),
        ("ntendon", c_int),
        ("nwrap", c_int),
        ("nsensor", c_int),
        ("nnumeric", c_int),
        ("nnumericdata", c_int),
        ("ntext", c_int),
        ("ntextdata", c_int),
        ("nkey", c_int),
        ("nuser_body", c_int),
        ("nuser_jnt", c_int),
        ("nuser_geom", c_int),
        ("nuser_site", c_int),
        ("nuser_tendon", c_int),
        ("nuser_actuator", c_int),
        ("nuser_sensor", c_int),
        ("nnames", c_int),
        ("nM", c_int),
        ("nemax", c_int),
        ("njmax", c_int),
        ("nconmax", c_int),
        ("nstack", c_int),
        ("nuserdata", c_int),
        ("nmocap", c_int),
        ("nsensordata", c_int),
        ("nbuffer", c_int),
        ("opt", MJOPTION),
        ("vis", MJVISUAL),
        ("stat", MJSTATISTIC),
        ("buffer", POINTER(c_ubyte)),
        ("qpos0", POINTER(c_double)),
        ("qpos_spring", POINTER(c_double)),
        ("body_parentid", POINTER(c_int)),
        ("body_rootid", POINTER(c_int)),
        ("body_weldid", POINTER(c_int)),
        ("body_mocapid", POINTER(c_int)),
        ("body_jntnum", POINTER(c_int)),
        ("body_jntadr", POINTER(c_int)),
        ("body_dofnum", POINTER(c_int)),
        ("body_dofadr", POINTER(c_int)),
        ("body_geomnum", POINTER(c_int)),
        ("body_geomadr", POINTER(c_int)),
        ("body_pos", POINTER(c_double)),
        ("body_quat", POINTER(c_double)),
        ("body_ipos", POINTER(c_double)),
        ("body_iquat", POINTER(c_double)),
        ("body_mass", POINTER(c_double)),
        ("body_inertia", POINTER(c_double)),
        ("body_invweight0", POINTER(c_double)),
        ("body_user", POINTER(c_double)),
        ("jnt_type", POINTER(c_int)),
        ("jnt_qposadr", POINTER(c_int)),
        ("jnt_dofadr", POINTER(c_int)),
        ("jnt_bodyid", POINTER(c_int)),
        ("jnt_limited", POINTER(c_ubyte)),
        ("jnt_solref", POINTER(c_double)),
        ("jnt_solimp", POINTER(c_double)),
        ("jnt_pos", POINTER(c_double)),
        ("jnt_axis", POINTER(c_double)),
        ("jnt_stiffness", POINTER(c_double)),
        ("jnt_range", POINTER(c_double)),
        ("jnt_margin", POINTER(c_double)),
        ("jnt_user", POINTER(c_double)),
        ("dof_bodyid", POINTER(c_int)),
        ("dof_jntid", POINTER(c_int)),
        ("dof_parentid", POINTER(c_int)),
        ("dof_Madr", POINTER(c_int)),
        ("dof_frictional", POINTER(c_ubyte)),
        ("dof_solref", POINTER(c_double)),
        ("dof_solimp", POINTER(c_double)),
        ("dof_frictionloss", POINTER(c_double)),
        ("dof_armature", POINTER(c_double)),
        ("dof_damping", POINTER(c_double)),
        ("dof_invweight0", POINTER(c_double)),
        ("geom_type", POINTER(c_int)),
        ("geom_contype", POINTER(c_int)),
        ("geom_conaffinity", POINTER(c_int)),
        ("geom_condim", POINTER(c_int)),
        ("geom_bodyid", POINTER(c_int)),
        ("geom_dataid", POINTER(c_int)),
        ("geom_matid", POINTER(c_int)),
        ("geom_group", POINTER(c_int)),
        ("geom_solmix", POINTER(c_double)),
        ("geom_solref", POINTER(c_double)),
        ("geom_solimp", POINTER(c_double)),
        ("geom_size", POINTER(c_double)),
        ("geom_rbound", POINTER(c_double)),
        ("geom_pos", POINTER(c_double)),
        ("geom_quat", POINTER(c_double)),
        ("geom_friction", POINTER(c_double)),
        ("geom_margin", POINTER(c_double)),
        ("geom_gap", POINTER(c_double)),
        ("geom_user", POINTER(c_double)),
        ("geom_rgba", POINTER(c_float)),
        ("site_type", POINTER(c_int)),
        ("site_bodyid", POINTER(c_int)),
        ("site_matid", POINTER(c_int)),
        ("site_group", POINTER(c_int)),
        ("site_size", POINTER(c_double)),
        ("site_pos", POINTER(c_double)),
        ("site_quat", POINTER(c_double)),
        ("site_user", POINTER(c_double)),
        ("site_rgba", POINTER(c_float)),
        ("cam_mode", POINTER(c_int)),
        ("cam_bodyid", POINTER(c_int)),
        ("cam_targetbodyid", POINTER(c_int)),
        ("cam_pos", POINTER(c_double)),
        ("cam_quat", POINTER(c_double)),
        ("cam_poscom0", POINTER(c_double)),
        ("cam_pos0", POINTER(c_double)),
        ("cam_mat0", POINTER(c_double)),
        ("cam_fovy", POINTER(c_double)),
        ("cam_ipd", POINTER(c_double)),
        ("light_mode", POINTER(c_int)),
        ("light_bodyid", POINTER(c_int)),
        ("light_targetbodyid", POINTER(c_int)),
        ("light_directional", POINTER(c_ubyte)),
        ("light_castshadow", POINTER(c_ubyte)),
        ("light_active", POINTER(c_ubyte)),
        ("light_pos", POINTER(c_double)),
        ("light_dir", POINTER(c_double)),
        ("light_poscom0", POINTER(c_double)),
        ("light_pos0", POINTER(c_double)),
        ("light_dir0", POINTER(c_double)),
        ("light_attenuation", POINTER(c_float)),
        ("light_cutoff", POINTER(c_float)),
        ("light_exponent", POINTER(c_float)),
        ("light_ambient", POINTER(c_float)),
        ("light_diffuse", POINTER(c_float)),
        ("light_specular", POINTER(c_float)),
        ("mesh_faceadr", POINTER(c_int)),
        ("mesh_facenum", POINTER(c_int)),
        ("mesh_vertadr", POINTER(c_int)),
        ("mesh_vertnum", POINTER(c_int)),
        ("mesh_graphadr", POINTER(c_int)),
        ("mesh_vert", POINTER(c_float)),
        ("mesh_normal", POINTER(c_float)),
        ("mesh_face", POINTER(c_int)),
        ("mesh_graph", POINTER(c_int)),
        ("hfield_size", POINTER(c_double)),
        ("hfield_nrow", POINTER(c_int)),
        ("hfield_ncol", POINTER(c_int)),
        ("hfield_adr", POINTER(c_int)),
        ("hfield_data", POINTER(c_float)),
        ("tex_type", POINTER(c_int)),
        ("tex_height", POINTER(c_int)),
        ("tex_width", POINTER(c_int)),
        ("tex_adr", POINTER(c_int)),
        ("tex_rgb", POINTER(c_ubyte)),
        ("mat_texid", POINTER(c_int)),
        ("mat_texuniform", POINTER(c_ubyte)),
        ("mat_texrepeat", POINTER(c_float)),
        ("mat_emission", POINTER(c_float)),
        ("mat_specular", POINTER(c_float)),
        ("mat_shininess", POINTER(c_float)),
        ("mat_reflectance", POINTER(c_float)),
        ("mat_rgba", POINTER(c_float)),
        ("pair_dim", POINTER(c_int)),
        ("pair_geom1", POINTER(c_int)),
        ("pair_geom2", POINTER(c_int)),
        ("pair_signature", POINTER(c_int)),
        ("pair_solref", POINTER(c_double)),
        ("pair_solimp", POINTER(c_double)),
        ("pair_margin", POINTER(c_double)),
        ("pair_gap", POINTER(c_double)),
        ("pair_friction", POINTER(c_double)),
        ("exclude_signature", POINTER(c_int)),
        ("eq_type", POINTER(c_int)),
        ("eq_obj1id", POINTER(c_int)),
        ("eq_obj2id", POINTER(c_int)),
        ("eq_active", POINTER(c_ubyte)),
        ("eq_solref", POINTER(c_double)),
        ("eq_solimp", POINTER(c_double)),
        ("eq_data", POINTER(c_double)),
        ("tendon_adr", POINTER(c_int)),
        ("tendon_num", POINTER(c_int)),
        ("tendon_matid", POINTER(c_int)),
        ("tendon_limited", POINTER(c_ubyte)),
        ("tendon_frictional", POINTER(c_ubyte)),
        ("tendon_width", POINTER(c_double)),
        ("tendon_solref_lim", POINTER(c_double)),
        ("tendon_solimp_lim", POINTER(c_double)),
        ("tendon_solref_fri", POINTER(c_double)),
        ("tendon_solimp_fri", POINTER(c_double)),
        ("tendon_range", POINTER(c_double)),
        ("tendon_margin", POINTER(c_double)),
        ("tendon_stiffness", POINTER(c_double)),
        ("tendon_damping", POINTER(c_double)),
        ("tendon_frictionloss", POINTER(c_double)),
        ("tendon_lengthspring", POINTER(c_double)),
        ("tendon_length0", POINTER(c_double)),
        ("tendon_invweight0", POINTER(c_double)),
        ("tendon_user", POINTER(c_double)),
        ("tendon_rgba", POINTER(c_float)),
        ("wrap_type", POINTER(c_int)),
        ("wrap_objid", POINTER(c_int)),
        ("wrap_prm", POINTER(c_double)),
        ("actuator_trntype", POINTER(c_int)),
        ("actuator_dyntype", POINTER(c_int)),
        ("actuator_gaintype", POINTER(c_int)),
        ("actuator_biastype", POINTER(c_int)),
        ("actuator_trnid", POINTER(c_int)),
        ("actuator_ctrllimited", POINTER(c_ubyte)),
        ("actuator_forcelimited", POINTER(c_ubyte)),
        ("actuator_dynprm", POINTER(c_double)),
        ("actuator_gainprm", POINTER(c_double)),
        ("actuator_biasprm", POINTER(c_double)),
        ("actuator_ctrlrange", POINTER(c_double)),
        ("actuator_forcerange", POINTER(c_double)),
        ("actuator_gear", POINTER(c_double)),
        ("actuator_cranklength", POINTER(c_double)),
        ("actuator_invweight0", POINTER(c_double)),
        ("actuator_length0", POINTER(c_double)),
        ("actuator_lengthrange", POINTER(c_double)),
        ("actuator_user", POINTER(c_double)),
        ("sensor_type", POINTER(c_int)),
        ("sensor_objid", POINTER(c_int)),
        ("sensor_dim", POINTER(c_int)),
        ("sensor_adr", POINTER(c_int)),
        ("sensor_scale", POINTER(c_double)),
        ("sensor_user", POINTER(c_double)),
        ("numeric_adr", POINTER(c_int)),
        ("numeric_size", POINTER(c_int)),
        ("numeric_data", POINTER(c_double)),
        ("text_adr", POINTER(c_int)),
        ("text_data", POINTER(c_char)),
        ("key_time", POINTER(c_double)),
        ("key_qpos", POINTER(c_double)),
        ("key_qvel", POINTER(c_double)),
        ("key_act", POINTER(c_double)),
        ("name_bodyadr", POINTER(c_int)),
        ("name_jntadr", POINTER(c_int)),
        ("name_geomadr", POINTER(c_int)),
        ("name_siteadr", POINTER(c_int)),
        ("name_camadr", POINTER(c_int)),
        ("name_lightadr", POINTER(c_int)),
        ("name_meshadr", POINTER(c_int)),
        ("name_hfieldadr", POINTER(c_int)),
        ("name_texadr", POINTER(c_int)),
        ("name_matadr", POINTER(c_int)),
        ("name_eqadr", POINTER(c_int)),
        ("name_tendonadr", POINTER(c_int)),
        ("name_actuatoradr", POINTER(c_int)),
        ("name_sensoradr", POINTER(c_int)),
        ("name_numericadr", POINTER(c_int)),
        ("name_textadr", POINTER(c_int)),
        ("names", POINTER(c_char)),
    ]

class MjContactWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def dist(self):
        return self._wrapped.contents.dist
    
    @dist.setter
    def dist(self, value):
        self._wrapped.contents.dist = value
    
    @property
    def pos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.pos, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @pos.setter
    def pos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.pos, val_ptr, 3 * sizeof(c_double))
    
    @property
    def frame(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.frame, dtype=np.double, count=(9)), (9, ))
        arr.setflags(write=False)
        return arr
    
    @frame.setter
    def frame(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.frame, val_ptr, 9 * sizeof(c_double))
    
    @property
    def includemargin(self):
        return self._wrapped.contents.includemargin
    
    @includemargin.setter
    def includemargin(self, value):
        self._wrapped.contents.includemargin = value
    
    @property
    def friction(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.friction, dtype=np.double, count=(5)), (5, ))
        arr.setflags(write=False)
        return arr
    
    @friction.setter
    def friction(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.friction, val_ptr, 5 * sizeof(c_double))
    
    @property
    def solref(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.solref, dtype=np.double, count=(2)), (2, ))
        arr.setflags(write=False)
        return arr
    
    @solref.setter
    def solref(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.solref, val_ptr, 2 * sizeof(c_double))
    
    @property
    def solimp(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.solimp, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @solimp.setter
    def solimp(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.solimp, val_ptr, 3 * sizeof(c_double))
    
    @property
    def mu(self):
        return self._wrapped.contents.mu
    
    @mu.setter
    def mu(self, value):
        self._wrapped.contents.mu = value
    
    @property
    def coef(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.coef, dtype=np.double, count=(5)), (5, ))
        arr.setflags(write=False)
        return arr
    
    @coef.setter
    def coef(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.coef, val_ptr, 5 * sizeof(c_double))
    
    @property
    def zone(self):
        return self._wrapped.contents.zone
    
    @zone.setter
    def zone(self, value):
        self._wrapped.contents.zone = value
    
    @property
    def dim(self):
        return self._wrapped.contents.dim
    
    @dim.setter
    def dim(self, value):
        self._wrapped.contents.dim = value
    
    @property
    def geom1(self):
        return self._wrapped.contents.geom1
    
    @geom1.setter
    def geom1(self, value):
        self._wrapped.contents.geom1 = value
    
    @property
    def geom2(self):
        return self._wrapped.contents.geom2
    
    @geom2.setter
    def geom2(self, value):
        self._wrapped.contents.geom2 = value
    
    @property
    def exclude(self):
        return self._wrapped.contents.exclude
    
    @exclude.setter
    def exclude(self, value):
        self._wrapped.contents.exclude = value
    
    @property
    def efc_address(self):
        return self._wrapped.contents.efc_address
    
    @efc_address.setter
    def efc_address(self, value):
        self._wrapped.contents.efc_address = value

class MjrRectWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def left(self):
        return self._wrapped.contents.left
    
    @left.setter
    def left(self, value):
        self._wrapped.contents.left = value
    
    @property
    def bottom(self):
        return self._wrapped.contents.bottom
    
    @bottom.setter
    def bottom(self, value):
        self._wrapped.contents.bottom = value
    
    @property
    def width(self):
        return self._wrapped.contents.width
    
    @width.setter
    def width(self, value):
        self._wrapped.contents.width = value
    
    @property
    def height(self):
        return self._wrapped.contents.height
    
    @height.setter
    def height(self, value):
        self._wrapped.contents.height = value

class MjvCameraPoseWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def head_pos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.head_pos, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @head_pos.setter
    def head_pos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.head_pos, val_ptr, 3 * sizeof(c_double))
    
    @property
    def head_right(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.head_right, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @head_right.setter
    def head_right(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.head_right, val_ptr, 3 * sizeof(c_double))
    
    @property
    def window_pos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.window_pos, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @window_pos.setter
    def window_pos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.window_pos, val_ptr, 3 * sizeof(c_double))
    
    @property
    def window_right(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.window_right, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @window_right.setter
    def window_right(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.window_right, val_ptr, 3 * sizeof(c_double))
    
    @property
    def window_up(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.window_up, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @window_up.setter
    def window_up(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.window_up, val_ptr, 3 * sizeof(c_double))
    
    @property
    def window_normal(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.window_normal, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @window_normal.setter
    def window_normal(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.window_normal, val_ptr, 3 * sizeof(c_double))
    
    @property
    def window_size(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.window_size, dtype=np.double, count=(2)), (2, ))
        arr.setflags(write=False)
        return arr
    
    @window_size.setter
    def window_size(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.window_size, val_ptr, 2 * sizeof(c_double))
    
    @property
    def scale(self):
        return self._wrapped.contents.scale
    
    @scale.setter
    def scale(self, value):
        self._wrapped.contents.scale = value
    
    @property
    def ipd(self):
        return self._wrapped.contents.ipd
    
    @ipd.setter
    def ipd(self, value):
        self._wrapped.contents.ipd = value

class MjrOptionWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def stereo(self):
        return self._wrapped.contents.stereo
    
    @stereo.setter
    def stereo(self, value):
        self._wrapped.contents.stereo = value
    
    @property
    def flags(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.flags, dtype=np.uint8, count=(6)), (6, ))
        arr.setflags(write=False)
        return arr
    
    @flags.setter
    def flags(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.flags, val_ptr, 6 * sizeof(c_ubyte))

class MjrContextWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def linewidth(self):
        return self._wrapped.contents.linewidth
    
    @linewidth.setter
    def linewidth(self, value):
        self._wrapped.contents.linewidth = value
    
    @property
    def znear(self):
        return self._wrapped.contents.znear
    
    @znear.setter
    def znear(self, value):
        self._wrapped.contents.znear = value
    
    @property
    def zfar(self):
        return self._wrapped.contents.zfar
    
    @zfar.setter
    def zfar(self, value):
        self._wrapped.contents.zfar = value
    
    @property
    def shadowclip(self):
        return self._wrapped.contents.shadowclip
    
    @shadowclip.setter
    def shadowclip(self, value):
        self._wrapped.contents.shadowclip = value
    
    @property
    def shadowscale(self):
        return self._wrapped.contents.shadowscale
    
    @shadowscale.setter
    def shadowscale(self, value):
        self._wrapped.contents.shadowscale = value
    
    @property
    def shadowsize(self):
        return self._wrapped.contents.shadowsize
    
    @shadowsize.setter
    def shadowsize(self, value):
        self._wrapped.contents.shadowsize = value
    
    @property
    def offwidth(self):
        return self._wrapped.contents.offwidth
    
    @offwidth.setter
    def offwidth(self, value):
        self._wrapped.contents.offwidth = value
    
    @property
    def offheight(self):
        return self._wrapped.contents.offheight
    
    @offheight.setter
    def offheight(self, value):
        self._wrapped.contents.offheight = value
    
    @property
    def offFBO(self):
        return self._wrapped.contents.offFBO
    
    @offFBO.setter
    def offFBO(self, value):
        self._wrapped.contents.offFBO = value
    
    @property
    def offColor(self):
        return self._wrapped.contents.offColor
    
    @offColor.setter
    def offColor(self, value):
        self._wrapped.contents.offColor = value
    
    @property
    def offDepthStencil(self):
        return self._wrapped.contents.offDepthStencil
    
    @offDepthStencil.setter
    def offDepthStencil(self, value):
        self._wrapped.contents.offDepthStencil = value
    
    @property
    def shadowFBO(self):
        return self._wrapped.contents.shadowFBO
    
    @shadowFBO.setter
    def shadowFBO(self, value):
        self._wrapped.contents.shadowFBO = value
    
    @property
    def shadowTex(self):
        return self._wrapped.contents.shadowTex
    
    @shadowTex.setter
    def shadowTex(self, value):
        self._wrapped.contents.shadowTex = value
    
    @property
    def ntexture(self):
        return self._wrapped.contents.ntexture
    
    @ntexture.setter
    def ntexture(self, value):
        self._wrapped.contents.ntexture = value
    
    @property
    def texture(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.texture, dtype=np.int, count=(100)), (100, ))
        arr.setflags(write=False)
        return arr
    
    @texture.setter
    def texture(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.texture, val_ptr, 100 * sizeof(c_int))
    
    @property
    def textureType(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.textureType, dtype=np.int, count=(100)), (100, ))
        arr.setflags(write=False)
        return arr
    
    @textureType.setter
    def textureType(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.textureType, val_ptr, 100 * sizeof(c_int))
    
    @property
    def basePlane(self):
        return self._wrapped.contents.basePlane
    
    @basePlane.setter
    def basePlane(self, value):
        self._wrapped.contents.basePlane = value
    
    @property
    def baseMesh(self):
        return self._wrapped.contents.baseMesh
    
    @baseMesh.setter
    def baseMesh(self, value):
        self._wrapped.contents.baseMesh = value
    
    @property
    def baseHField(self):
        return self._wrapped.contents.baseHField
    
    @baseHField.setter
    def baseHField(self, value):
        self._wrapped.contents.baseHField = value
    
    @property
    def baseBuiltin(self):
        return self._wrapped.contents.baseBuiltin
    
    @baseBuiltin.setter
    def baseBuiltin(self, value):
        self._wrapped.contents.baseBuiltin = value
    
    @property
    def baseFontNormal(self):
        return self._wrapped.contents.baseFontNormal
    
    @baseFontNormal.setter
    def baseFontNormal(self, value):
        self._wrapped.contents.baseFontNormal = value
    
    @property
    def baseFontBack(self):
        return self._wrapped.contents.baseFontBack
    
    @baseFontBack.setter
    def baseFontBack(self, value):
        self._wrapped.contents.baseFontBack = value
    
    @property
    def baseFontBig(self):
        return self._wrapped.contents.baseFontBig
    
    @baseFontBig.setter
    def baseFontBig(self, value):
        self._wrapped.contents.baseFontBig = value
    
    @property
    def rangePlane(self):
        return self._wrapped.contents.rangePlane
    
    @rangePlane.setter
    def rangePlane(self, value):
        self._wrapped.contents.rangePlane = value
    
    @property
    def rangeMesh(self):
        return self._wrapped.contents.rangeMesh
    
    @rangeMesh.setter
    def rangeMesh(self, value):
        self._wrapped.contents.rangeMesh = value
    
    @property
    def rangeHField(self):
        return self._wrapped.contents.rangeHField
    
    @rangeHField.setter
    def rangeHField(self, value):
        self._wrapped.contents.rangeHField = value
    
    @property
    def rangeBuiltin(self):
        return self._wrapped.contents.rangeBuiltin
    
    @rangeBuiltin.setter
    def rangeBuiltin(self, value):
        self._wrapped.contents.rangeBuiltin = value
    
    @property
    def rangeFont(self):
        return self._wrapped.contents.rangeFont
    
    @rangeFont.setter
    def rangeFont(self, value):
        self._wrapped.contents.rangeFont = value
    
    @property
    def charWidth(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.charWidth, dtype=np.int, count=(127)), (127, ))
        arr.setflags(write=False)
        return arr
    
    @charWidth.setter
    def charWidth(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.charWidth, val_ptr, 127 * sizeof(c_int))
    
    @property
    def charWidthBig(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.charWidthBig, dtype=np.int, count=(127)), (127, ))
        arr.setflags(write=False)
        return arr
    
    @charWidthBig.setter
    def charWidthBig(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.charWidthBig, val_ptr, 127 * sizeof(c_int))
    
    @property
    def charHeight(self):
        return self._wrapped.contents.charHeight
    
    @charHeight.setter
    def charHeight(self, value):
        self._wrapped.contents.charHeight = value
    
    @property
    def charHeightBig(self):
        return self._wrapped.contents.charHeightBig
    
    @charHeightBig.setter
    def charHeightBig(self, value):
        self._wrapped.contents.charHeightBig = value
    
    @property
    def glewInitialized(self):
        return self._wrapped.contents.glewInitialized
    
    @glewInitialized.setter
    def glewInitialized(self, value):
        self._wrapped.contents.glewInitialized = value

class MjvCameraWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def fovy(self):
        return self._wrapped.contents.fovy
    
    @fovy.setter
    def fovy(self, value):
        self._wrapped.contents.fovy = value
    
    @property
    def camid(self):
        return self._wrapped.contents.camid
    
    @camid.setter
    def camid(self, value):
        self._wrapped.contents.camid = value
    
    @property
    def trackbodyid(self):
        return self._wrapped.contents.trackbodyid
    
    @trackbodyid.setter
    def trackbodyid(self, value):
        self._wrapped.contents.trackbodyid = value
    
    @property
    def lookat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.lookat, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @lookat.setter
    def lookat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.lookat, val_ptr, 3 * sizeof(c_double))
    
    @property
    def azimuth(self):
        return self._wrapped.contents.azimuth
    
    @azimuth.setter
    def azimuth(self, value):
        self._wrapped.contents.azimuth = value
    
    @property
    def elevation(self):
        return self._wrapped.contents.elevation
    
    @elevation.setter
    def elevation(self, value):
        self._wrapped.contents.elevation = value
    
    @property
    def distance(self):
        return self._wrapped.contents.distance
    
    @distance.setter
    def distance(self, value):
        self._wrapped.contents.distance = value
    
    @property
    def pose(self):
        return self._wrapped.contents.pose
    
    @pose.setter
    def pose(self, value):
        self._wrapped.contents.pose = value
    
    @property
    def VR(self):
        return self._wrapped.contents.VR
    
    @VR.setter
    def VR(self, value):
        self._wrapped.contents.VR = value

class MjvOptionWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def label(self):
        return self._wrapped.contents.label
    
    @label.setter
    def label(self, value):
        self._wrapped.contents.label = value
    
    @property
    def frame(self):
        return self._wrapped.contents.frame
    
    @frame.setter
    def frame(self, value):
        self._wrapped.contents.frame = value
    
    @property
    def geomgroup(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geomgroup, dtype=np.uint8, count=(5)), (5, ))
        arr.setflags(write=False)
        return arr
    
    @geomgroup.setter
    def geomgroup(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.geomgroup, val_ptr, 5 * sizeof(c_ubyte))
    
    @property
    def sitegroup(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.sitegroup, dtype=np.uint8, count=(5)), (5, ))
        arr.setflags(write=False)
        return arr
    
    @sitegroup.setter
    def sitegroup(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.sitegroup, val_ptr, 5 * sizeof(c_ubyte))
    
    @property
    def flags(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.flags, dtype=np.uint8, count=(18)), (18, ))
        arr.setflags(write=False)
        return arr
    
    @flags.setter
    def flags(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.flags, val_ptr, 18 * sizeof(c_ubyte))

class MjvGeomWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def type(self):
        return self._wrapped.contents.type
    
    @type.setter
    def type(self, value):
        self._wrapped.contents.type = value
    
    @property
    def dataid(self):
        return self._wrapped.contents.dataid
    
    @dataid.setter
    def dataid(self, value):
        self._wrapped.contents.dataid = value
    
    @property
    def objtype(self):
        return self._wrapped.contents.objtype
    
    @objtype.setter
    def objtype(self, value):
        self._wrapped.contents.objtype = value
    
    @property
    def objid(self):
        return self._wrapped.contents.objid
    
    @objid.setter
    def objid(self, value):
        self._wrapped.contents.objid = value
    
    @property
    def category(self):
        return self._wrapped.contents.category
    
    @category.setter
    def category(self, value):
        self._wrapped.contents.category = value
    
    @property
    def texid(self):
        return self._wrapped.contents.texid
    
    @texid.setter
    def texid(self, value):
        self._wrapped.contents.texid = value
    
    @property
    def texuniform(self):
        return self._wrapped.contents.texuniform
    
    @texuniform.setter
    def texuniform(self, value):
        self._wrapped.contents.texuniform = value
    
    @property
    def texrepeat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.texrepeat, dtype=np.float, count=(2)), (2, ))
        arr.setflags(write=False)
        return arr
    
    @texrepeat.setter
    def texrepeat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.texrepeat, val_ptr, 2 * sizeof(c_float))
    
    @property
    def size(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.size, dtype=np.float, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @size.setter
    def size(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.size, val_ptr, 3 * sizeof(c_float))
    
    @property
    def pos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.pos, dtype=np.float, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @pos.setter
    def pos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.pos, val_ptr, 3 * sizeof(c_float))
    
    @property
    def mat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mat, dtype=np.float, count=(9)), (9, ))
        arr.setflags(write=False)
        return arr
    
    @mat.setter
    def mat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.mat, val_ptr, 9 * sizeof(c_float))
    
    @property
    def rgba(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.rgba, dtype=np.float, count=(4)), (4, ))
        arr.setflags(write=False)
        return arr
    
    @rgba.setter
    def rgba(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.rgba, val_ptr, 4 * sizeof(c_float))
    
    @property
    def emission(self):
        return self._wrapped.contents.emission
    
    @emission.setter
    def emission(self, value):
        self._wrapped.contents.emission = value
    
    @property
    def specular(self):
        return self._wrapped.contents.specular
    
    @specular.setter
    def specular(self, value):
        self._wrapped.contents.specular = value
    
    @property
    def shininess(self):
        return self._wrapped.contents.shininess
    
    @shininess.setter
    def shininess(self, value):
        self._wrapped.contents.shininess = value
    
    @property
    def reflectance(self):
        return self._wrapped.contents.reflectance
    
    @reflectance.setter
    def reflectance(self, value):
        self._wrapped.contents.reflectance = value
    
    @property
    def label(self):
        return self._wrapped.contents.label
    
    @label.setter
    def label(self, value):
        self._wrapped.contents.label = value
    
    @property
    def camdist(self):
        return self._wrapped.contents.camdist
    
    @camdist.setter
    def camdist(self, value):
        self._wrapped.contents.camdist = value
    
    @property
    def rbound(self):
        return self._wrapped.contents.rbound
    
    @rbound.setter
    def rbound(self, value):
        self._wrapped.contents.rbound = value
    
    @property
    def transparent(self):
        return self._wrapped.contents.transparent
    
    @transparent.setter
    def transparent(self, value):
        self._wrapped.contents.transparent = value

class MjvLightWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def pos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.pos, dtype=np.float, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @pos.setter
    def pos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.pos, val_ptr, 3 * sizeof(c_float))
    
    @property
    def dir(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.dir, dtype=np.float, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @dir.setter
    def dir(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.dir, val_ptr, 3 * sizeof(c_float))
    
    @property
    def attenuation(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.attenuation, dtype=np.float, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @attenuation.setter
    def attenuation(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.attenuation, val_ptr, 3 * sizeof(c_float))
    
    @property
    def cutoff(self):
        return self._wrapped.contents.cutoff
    
    @cutoff.setter
    def cutoff(self, value):
        self._wrapped.contents.cutoff = value
    
    @property
    def exponent(self):
        return self._wrapped.contents.exponent
    
    @exponent.setter
    def exponent(self, value):
        self._wrapped.contents.exponent = value
    
    @property
    def ambient(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.ambient, dtype=np.float, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @ambient.setter
    def ambient(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.ambient, val_ptr, 3 * sizeof(c_float))
    
    @property
    def diffuse(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.diffuse, dtype=np.float, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @diffuse.setter
    def diffuse(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.diffuse, val_ptr, 3 * sizeof(c_float))
    
    @property
    def specular(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.specular, dtype=np.float, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @specular.setter
    def specular(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.specular, val_ptr, 3 * sizeof(c_float))
    
    @property
    def headlight(self):
        return self._wrapped.contents.headlight
    
    @headlight.setter
    def headlight(self, value):
        self._wrapped.contents.headlight = value
    
    @property
    def directional(self):
        return self._wrapped.contents.directional
    
    @directional.setter
    def directional(self, value):
        self._wrapped.contents.directional = value
    
    @property
    def castshadow(self):
        return self._wrapped.contents.castshadow
    
    @castshadow.setter
    def castshadow(self, value):
        self._wrapped.contents.castshadow = value

class MjvObjectsWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def nlight(self):
        return self._wrapped.contents.nlight
    
    @nlight.setter
    def nlight(self, value):
        self._wrapped.contents.nlight = value
    
    @property
    def ngeom(self):
        return self._wrapped.contents.ngeom
    
    @ngeom.setter
    def ngeom(self, value):
        self._wrapped.contents.ngeom = value
    
    @property
    def maxgeom(self):
        return self._wrapped.contents.maxgeom
    
    @maxgeom.setter
    def maxgeom(self, value):
        self._wrapped.contents.maxgeom = value
    
    @property
    def lights(self):
        return self._wrapped.contents.lights
    
    @lights.setter
    def lights(self, value):
        self._wrapped.contents.lights = value

class MjOptionWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def timestep(self):
        return self._wrapped.contents.timestep
    
    @timestep.setter
    def timestep(self, value):
        self._wrapped.contents.timestep = value
    
    @property
    def apirate(self):
        return self._wrapped.contents.apirate
    
    @apirate.setter
    def apirate(self, value):
        self._wrapped.contents.apirate = value
    
    @property
    def tolerance(self):
        return self._wrapped.contents.tolerance
    
    @tolerance.setter
    def tolerance(self, value):
        self._wrapped.contents.tolerance = value
    
    @property
    def impratio(self):
        return self._wrapped.contents.impratio
    
    @impratio.setter
    def impratio(self, value):
        self._wrapped.contents.impratio = value
    
    @property
    def gravity(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.gravity, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @gravity.setter
    def gravity(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.gravity, val_ptr, 3 * sizeof(c_double))
    
    @property
    def wind(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.wind, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @wind.setter
    def wind(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.wind, val_ptr, 3 * sizeof(c_double))
    
    @property
    def magnetic(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.magnetic, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @magnetic.setter
    def magnetic(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.magnetic, val_ptr, 3 * sizeof(c_double))
    
    @property
    def density(self):
        return self._wrapped.contents.density
    
    @density.setter
    def density(self, value):
        self._wrapped.contents.density = value
    
    @property
    def viscosity(self):
        return self._wrapped.contents.viscosity
    
    @viscosity.setter
    def viscosity(self, value):
        self._wrapped.contents.viscosity = value
    
    @property
    def o_margin(self):
        return self._wrapped.contents.o_margin
    
    @o_margin.setter
    def o_margin(self, value):
        self._wrapped.contents.o_margin = value
    
    @property
    def o_solref(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.o_solref, dtype=np.double, count=(2)), (2, ))
        arr.setflags(write=False)
        return arr
    
    @o_solref.setter
    def o_solref(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.o_solref, val_ptr, 2 * sizeof(c_double))
    
    @property
    def o_solimp(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.o_solimp, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @o_solimp.setter
    def o_solimp(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.o_solimp, val_ptr, 3 * sizeof(c_double))
    
    @property
    def mpr_tolerance(self):
        return self._wrapped.contents.mpr_tolerance
    
    @mpr_tolerance.setter
    def mpr_tolerance(self, value):
        self._wrapped.contents.mpr_tolerance = value
    
    @property
    def mpr_iterations(self):
        return self._wrapped.contents.mpr_iterations
    
    @mpr_iterations.setter
    def mpr_iterations(self, value):
        self._wrapped.contents.mpr_iterations = value
    
    @property
    def integrator(self):
        return self._wrapped.contents.integrator
    
    @integrator.setter
    def integrator(self, value):
        self._wrapped.contents.integrator = value
    
    @property
    def collision(self):
        return self._wrapped.contents.collision
    
    @collision.setter
    def collision(self, value):
        self._wrapped.contents.collision = value
    
    @property
    def impedance(self):
        return self._wrapped.contents.impedance
    
    @impedance.setter
    def impedance(self, value):
        self._wrapped.contents.impedance = value
    
    @property
    def reference(self):
        return self._wrapped.contents.reference
    
    @reference.setter
    def reference(self, value):
        self._wrapped.contents.reference = value
    
    @property
    def solver(self):
        return self._wrapped.contents.solver
    
    @solver.setter
    def solver(self, value):
        self._wrapped.contents.solver = value
    
    @property
    def iterations(self):
        return self._wrapped.contents.iterations
    
    @iterations.setter
    def iterations(self, value):
        self._wrapped.contents.iterations = value
    
    @property
    def disableflags(self):
        return self._wrapped.contents.disableflags
    
    @disableflags.setter
    def disableflags(self, value):
        self._wrapped.contents.disableflags = value
    
    @property
    def enableflags(self):
        return self._wrapped.contents.enableflags
    
    @enableflags.setter
    def enableflags(self, value):
        self._wrapped.contents.enableflags = value

class MjVisualWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def global_(self):
        return self._wrapped.contents.global_
    
    @global_.setter
    def global_(self, value):
        self._wrapped.contents.global_ = value
    
    @property
    def quality(self):
        return self._wrapped.contents.quality
    
    @quality.setter
    def quality(self, value):
        self._wrapped.contents.quality = value
    
    @property
    def headlight(self):
        return self._wrapped.contents.headlight
    
    @headlight.setter
    def headlight(self, value):
        self._wrapped.contents.headlight = value
    
    @property
    def map_(self):
        return self._wrapped.contents.map_
    
    @map_.setter
    def map_(self, value):
        self._wrapped.contents.map_ = value
    
    @property
    def scale(self):
        return self._wrapped.contents.scale
    
    @scale.setter
    def scale(self, value):
        self._wrapped.contents.scale = value
    
    @property
    def rgba(self):
        return self._wrapped.contents.rgba
    
    @rgba.setter
    def rgba(self, value):
        self._wrapped.contents.rgba = value

class MjStatisticWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def meanmass(self):
        return self._wrapped.contents.meanmass
    
    @meanmass.setter
    def meanmass(self, value):
        self._wrapped.contents.meanmass = value
    
    @property
    def meansize(self):
        return self._wrapped.contents.meansize
    
    @meansize.setter
    def meansize(self, value):
        self._wrapped.contents.meansize = value
    
    @property
    def extent(self):
        return self._wrapped.contents.extent
    
    @extent.setter
    def extent(self, value):
        self._wrapped.contents.extent = value
    
    @property
    def center(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.center, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @center.setter
    def center(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.center, val_ptr, 3 * sizeof(c_double))

class MjDataWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def nstack(self):
        return self._wrapped.contents.nstack
    
    @nstack.setter
    def nstack(self, value):
        self._wrapped.contents.nstack = value
    
    @property
    def nbuffer(self):
        return self._wrapped.contents.nbuffer
    
    @nbuffer.setter
    def nbuffer(self, value):
        self._wrapped.contents.nbuffer = value
    
    @property
    def pstack(self):
        return self._wrapped.contents.pstack
    
    @pstack.setter
    def pstack(self, value):
        self._wrapped.contents.pstack = value
    
    @property
    def maxstackuse(self):
        return self._wrapped.contents.maxstackuse
    
    @maxstackuse.setter
    def maxstackuse(self, value):
        self._wrapped.contents.maxstackuse = value
    
    @property
    def ne(self):
        return self._wrapped.contents.ne
    
    @ne.setter
    def ne(self, value):
        self._wrapped.contents.ne = value
    
    @property
    def nf(self):
        return self._wrapped.contents.nf
    
    @nf.setter
    def nf(self, value):
        self._wrapped.contents.nf = value
    
    @property
    def nefc(self):
        return self._wrapped.contents.nefc
    
    @nefc.setter
    def nefc(self, value):
        self._wrapped.contents.nefc = value
    
    @property
    def ncon(self):
        return self._wrapped.contents.ncon
    
    @ncon.setter
    def ncon(self, value):
        self._wrapped.contents.ncon = value
    
    @property
    def nwarning(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.nwarning, dtype=np.int, count=(8)), (8, ))
        arr.setflags(write=False)
        return arr
    
    @nwarning.setter
    def nwarning(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.nwarning, val_ptr, 8 * sizeof(c_int))
    
    @property
    def warning_info(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.warning_info, dtype=np.int, count=(8)), (8, ))
        arr.setflags(write=False)
        return arr
    
    @warning_info.setter
    def warning_info(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.warning_info, val_ptr, 8 * sizeof(c_int))
    
    @property
    def timer_duration(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.timer_duration, dtype=np.double, count=(14)), (14, ))
        arr.setflags(write=False)
        return arr
    
    @timer_duration.setter
    def timer_duration(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.timer_duration, val_ptr, 14 * sizeof(c_double))
    
    @property
    def timer_ncall(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.timer_ncall, dtype=np.double, count=(14)), (14, ))
        arr.setflags(write=False)
        return arr
    
    @timer_ncall.setter
    def timer_ncall(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.timer_ncall, val_ptr, 14 * sizeof(c_double))
    
    @property
    def mocaptime(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mocaptime, dtype=np.double, count=(3)), (3, ))
        arr.setflags(write=False)
        return arr
    
    @mocaptime.setter
    def mocaptime(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.mocaptime, val_ptr, 3 * sizeof(c_double))
    
    @property
    def time(self):
        return self._wrapped.contents.time
    
    @time.setter
    def time(self, value):
        self._wrapped.contents.time = value
    
    @property
    def energy(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.energy, dtype=np.double, count=(2)), (2, ))
        arr.setflags(write=False)
        return arr
    
    @energy.setter
    def energy(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.energy, val_ptr, 2 * sizeof(c_double))
    
    @property
    def solverstat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.solverstat, dtype=np.double, count=(4)), (4, ))
        arr.setflags(write=False)
        return arr
    
    @solverstat.setter
    def solverstat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.solverstat, val_ptr, 4 * sizeof(c_double))
    
    @property
    def solvertrace(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.solvertrace, dtype=np.double, count=(200)), (200, ))
        arr.setflags(write=False)
        return arr
    
    @solvertrace.setter
    def solvertrace(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.solvertrace, val_ptr, 200 * sizeof(c_double))
    
    @property
    def buffer(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.buffer, dtype=np.uint8, count=(self.nbuffer)), (self.nbuffer, ))
        arr.setflags(write=False)
        return arr
    
    @buffer.setter
    def buffer(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.buffer, val_ptr, self.nbuffer * sizeof(c_ubyte))
    
    @property
    def stack(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.stack, dtype=np.double, count=(self.nstack)), (self.nstack, ))
        arr.setflags(write=False)
        return arr
    
    @stack.setter
    def stack(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.stack, val_ptr, self.nstack * sizeof(c_double))
    
    @property
    def qpos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qpos, dtype=np.double, count=(self._size_src.nq*1)), (self._size_src.nq, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qpos.setter
    def qpos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qpos, val_ptr, self._size_src.nq*1 * sizeof(c_double))
    
    @property
    def qvel(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qvel, dtype=np.double, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qvel.setter
    def qvel(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qvel, val_ptr, self._size_src.nv*1 * sizeof(c_double))
    
    @property
    def act(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.act, dtype=np.double, count=(self._size_src.na*1)), (self._size_src.na, 1, ))
        arr.setflags(write=False)
        return arr
    
    @act.setter
    def act(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.act, val_ptr, self._size_src.na*1 * sizeof(c_double))
    
    @property
    def ctrl(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.ctrl, dtype=np.double, count=(self._size_src.nu*1)), (self._size_src.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @ctrl.setter
    def ctrl(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.ctrl, val_ptr, self._size_src.nu*1 * sizeof(c_double))
    
    @property
    def qfrc_applied(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qfrc_applied, dtype=np.double, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qfrc_applied.setter
    def qfrc_applied(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qfrc_applied, val_ptr, self._size_src.nv*1 * sizeof(c_double))
    
    @property
    def xfrc_applied(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.xfrc_applied, dtype=np.double, count=(self._size_src.nbody*6)), (self._size_src.nbody, 6, ))
        arr.setflags(write=False)
        return arr
    
    @xfrc_applied.setter
    def xfrc_applied(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.xfrc_applied, val_ptr, self._size_src.nbody*6 * sizeof(c_double))
    
    @property
    def qacc(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qacc, dtype=np.double, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qacc.setter
    def qacc(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qacc, val_ptr, self._size_src.nv*1 * sizeof(c_double))
    
    @property
    def act_dot(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.act_dot, dtype=np.double, count=(self._size_src.na*1)), (self._size_src.na, 1, ))
        arr.setflags(write=False)
        return arr
    
    @act_dot.setter
    def act_dot(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.act_dot, val_ptr, self._size_src.na*1 * sizeof(c_double))
    
    @property
    def mocap_pos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mocap_pos, dtype=np.double, count=(self._size_src.nmocap*3)), (self._size_src.nmocap, 3, ))
        arr.setflags(write=False)
        return arr
    
    @mocap_pos.setter
    def mocap_pos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.mocap_pos, val_ptr, self._size_src.nmocap*3 * sizeof(c_double))
    
    @property
    def mocap_quat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mocap_quat, dtype=np.double, count=(self._size_src.nmocap*4)), (self._size_src.nmocap, 4, ))
        arr.setflags(write=False)
        return arr
    
    @mocap_quat.setter
    def mocap_quat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.mocap_quat, val_ptr, self._size_src.nmocap*4 * sizeof(c_double))
    
    @property
    def userdata(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.userdata, dtype=np.double, count=(self._size_src.nuserdata*1)), (self._size_src.nuserdata, 1, ))
        arr.setflags(write=False)
        return arr
    
    @userdata.setter
    def userdata(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.userdata, val_ptr, self._size_src.nuserdata*1 * sizeof(c_double))
    
    @property
    def sensordata(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.sensordata, dtype=np.double, count=(self._size_src.nsensordata*1)), (self._size_src.nsensordata, 1, ))
        arr.setflags(write=False)
        return arr
    
    @sensordata.setter
    def sensordata(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.sensordata, val_ptr, self._size_src.nsensordata*1 * sizeof(c_double))
    
    @property
    def xpos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.xpos, dtype=np.double, count=(self._size_src.nbody*3)), (self._size_src.nbody, 3, ))
        arr.setflags(write=False)
        return arr
    
    @xpos.setter
    def xpos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.xpos, val_ptr, self._size_src.nbody*3 * sizeof(c_double))
    
    @property
    def xquat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.xquat, dtype=np.double, count=(self._size_src.nbody*4)), (self._size_src.nbody, 4, ))
        arr.setflags(write=False)
        return arr
    
    @xquat.setter
    def xquat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.xquat, val_ptr, self._size_src.nbody*4 * sizeof(c_double))
    
    @property
    def xmat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.xmat, dtype=np.double, count=(self._size_src.nbody*9)), (self._size_src.nbody, 9, ))
        arr.setflags(write=False)
        return arr
    
    @xmat.setter
    def xmat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.xmat, val_ptr, self._size_src.nbody*9 * sizeof(c_double))
    
    @property
    def xipos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.xipos, dtype=np.double, count=(self._size_src.nbody*3)), (self._size_src.nbody, 3, ))
        arr.setflags(write=False)
        return arr
    
    @xipos.setter
    def xipos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.xipos, val_ptr, self._size_src.nbody*3 * sizeof(c_double))
    
    @property
    def ximat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.ximat, dtype=np.double, count=(self._size_src.nbody*9)), (self._size_src.nbody, 9, ))
        arr.setflags(write=False)
        return arr
    
    @ximat.setter
    def ximat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.ximat, val_ptr, self._size_src.nbody*9 * sizeof(c_double))
    
    @property
    def xanchor(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.xanchor, dtype=np.double, count=(self._size_src.njnt*3)), (self._size_src.njnt, 3, ))
        arr.setflags(write=False)
        return arr
    
    @xanchor.setter
    def xanchor(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.xanchor, val_ptr, self._size_src.njnt*3 * sizeof(c_double))
    
    @property
    def xaxis(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.xaxis, dtype=np.double, count=(self._size_src.njnt*3)), (self._size_src.njnt, 3, ))
        arr.setflags(write=False)
        return arr
    
    @xaxis.setter
    def xaxis(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.xaxis, val_ptr, self._size_src.njnt*3 * sizeof(c_double))
    
    @property
    def geom_xpos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_xpos, dtype=np.double, count=(self._size_src.ngeom*3)), (self._size_src.ngeom, 3, ))
        arr.setflags(write=False)
        return arr
    
    @geom_xpos.setter
    def geom_xpos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.geom_xpos, val_ptr, self._size_src.ngeom*3 * sizeof(c_double))
    
    @property
    def geom_xmat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_xmat, dtype=np.double, count=(self._size_src.ngeom*9)), (self._size_src.ngeom, 9, ))
        arr.setflags(write=False)
        return arr
    
    @geom_xmat.setter
    def geom_xmat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.geom_xmat, val_ptr, self._size_src.ngeom*9 * sizeof(c_double))
    
    @property
    def site_xpos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.site_xpos, dtype=np.double, count=(self._size_src.nsite*3)), (self._size_src.nsite, 3, ))
        arr.setflags(write=False)
        return arr
    
    @site_xpos.setter
    def site_xpos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.site_xpos, val_ptr, self._size_src.nsite*3 * sizeof(c_double))
    
    @property
    def site_xmat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.site_xmat, dtype=np.double, count=(self._size_src.nsite*9)), (self._size_src.nsite, 9, ))
        arr.setflags(write=False)
        return arr
    
    @site_xmat.setter
    def site_xmat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.site_xmat, val_ptr, self._size_src.nsite*9 * sizeof(c_double))
    
    @property
    def cam_xpos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cam_xpos, dtype=np.double, count=(self._size_src.ncam*3)), (self._size_src.ncam, 3, ))
        arr.setflags(write=False)
        return arr
    
    @cam_xpos.setter
    def cam_xpos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cam_xpos, val_ptr, self._size_src.ncam*3 * sizeof(c_double))
    
    @property
    def cam_xmat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cam_xmat, dtype=np.double, count=(self._size_src.ncam*9)), (self._size_src.ncam, 9, ))
        arr.setflags(write=False)
        return arr
    
    @cam_xmat.setter
    def cam_xmat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cam_xmat, val_ptr, self._size_src.ncam*9 * sizeof(c_double))
    
    @property
    def light_xpos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_xpos, dtype=np.double, count=(self._size_src.nlight*3)), (self._size_src.nlight, 3, ))
        arr.setflags(write=False)
        return arr
    
    @light_xpos.setter
    def light_xpos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.light_xpos, val_ptr, self._size_src.nlight*3 * sizeof(c_double))
    
    @property
    def light_xdir(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_xdir, dtype=np.double, count=(self._size_src.nlight*3)), (self._size_src.nlight, 3, ))
        arr.setflags(write=False)
        return arr
    
    @light_xdir.setter
    def light_xdir(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.light_xdir, val_ptr, self._size_src.nlight*3 * sizeof(c_double))
    
    @property
    def com_subtree(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.com_subtree, dtype=np.double, count=(self._size_src.nbody*3)), (self._size_src.nbody, 3, ))
        arr.setflags(write=False)
        return arr
    
    @com_subtree.setter
    def com_subtree(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.com_subtree, val_ptr, self._size_src.nbody*3 * sizeof(c_double))
    
    @property
    def cdof(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cdof, dtype=np.double, count=(self._size_src.nv*6)), (self._size_src.nv, 6, ))
        arr.setflags(write=False)
        return arr
    
    @cdof.setter
    def cdof(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cdof, val_ptr, self._size_src.nv*6 * sizeof(c_double))
    
    @property
    def cinert(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cinert, dtype=np.double, count=(self._size_src.nbody*10)), (self._size_src.nbody, 10, ))
        arr.setflags(write=False)
        return arr
    
    @cinert.setter
    def cinert(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cinert, val_ptr, self._size_src.nbody*10 * sizeof(c_double))
    
    @property
    def ten_wrapadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.ten_wrapadr, dtype=np.int, count=(self._size_src.ntendon*1)), (self._size_src.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @ten_wrapadr.setter
    def ten_wrapadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.ten_wrapadr, val_ptr, self._size_src.ntendon*1 * sizeof(c_int))
    
    @property
    def ten_wrapnum(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.ten_wrapnum, dtype=np.int, count=(self._size_src.ntendon*1)), (self._size_src.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @ten_wrapnum.setter
    def ten_wrapnum(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.ten_wrapnum, val_ptr, self._size_src.ntendon*1 * sizeof(c_int))
    
    @property
    def ten_length(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.ten_length, dtype=np.double, count=(self._size_src.ntendon*1)), (self._size_src.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @ten_length.setter
    def ten_length(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.ten_length, val_ptr, self._size_src.ntendon*1 * sizeof(c_double))
    
    @property
    def ten_moment(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.ten_moment, dtype=np.double, count=(self._size_src.ntendon*self._size_src.nv)), (self._size_src.ntendon, self._size_src.nv, ))
        arr.setflags(write=False)
        return arr
    
    @ten_moment.setter
    def ten_moment(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.ten_moment, val_ptr, self._size_src.ntendon*self._size_src.nv * sizeof(c_double))
    
    @property
    def wrap_obj(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.wrap_obj, dtype=np.int, count=(self._size_src.nwrap*2)), (self._size_src.nwrap, 2, ))
        arr.setflags(write=False)
        return arr
    
    @wrap_obj.setter
    def wrap_obj(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.wrap_obj, val_ptr, self._size_src.nwrap*2 * sizeof(c_int))
    
    @property
    def wrap_xpos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.wrap_xpos, dtype=np.double, count=(self._size_src.nwrap*6)), (self._size_src.nwrap, 6, ))
        arr.setflags(write=False)
        return arr
    
    @wrap_xpos.setter
    def wrap_xpos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.wrap_xpos, val_ptr, self._size_src.nwrap*6 * sizeof(c_double))
    
    @property
    def actuator_length(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_length, dtype=np.double, count=(self._size_src.nu*1)), (self._size_src.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_length.setter
    def actuator_length(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_length, val_ptr, self._size_src.nu*1 * sizeof(c_double))
    
    @property
    def actuator_moment(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_moment, dtype=np.double, count=(self._size_src.nu*self._size_src.nv)), (self._size_src.nu, self._size_src.nv, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_moment.setter
    def actuator_moment(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_moment, val_ptr, self._size_src.nu*self._size_src.nv * sizeof(c_double))
    
    @property
    def crb(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.crb, dtype=np.double, count=(self._size_src.nbody*10)), (self._size_src.nbody, 10, ))
        arr.setflags(write=False)
        return arr
    
    @crb.setter
    def crb(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.crb, val_ptr, self._size_src.nbody*10 * sizeof(c_double))
    
    @property
    def qM(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qM, dtype=np.double, count=(self._size_src.nM*1)), (self._size_src.nM, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qM.setter
    def qM(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qM, val_ptr, self._size_src.nM*1 * sizeof(c_double))
    
    @property
    def qLD(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qLD, dtype=np.double, count=(self._size_src.nM*1)), (self._size_src.nM, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qLD.setter
    def qLD(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qLD, val_ptr, self._size_src.nM*1 * sizeof(c_double))
    
    @property
    def qLDiagInv(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qLDiagInv, dtype=np.double, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qLDiagInv.setter
    def qLDiagInv(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qLDiagInv, val_ptr, self._size_src.nv*1 * sizeof(c_double))
    
    @property
    def qLDiagSqrtInv(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qLDiagSqrtInv, dtype=np.double, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qLDiagSqrtInv.setter
    def qLDiagSqrtInv(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qLDiagSqrtInv, val_ptr, self._size_src.nv*1 * sizeof(c_double))
    
    @property
    def efc_type(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_type, dtype=np.int, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_type.setter
    def efc_type(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.efc_type, val_ptr, self._size_src.njmax*1 * sizeof(c_int))
    
    @property
    def efc_id(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_id, dtype=np.int, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_id.setter
    def efc_id(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.efc_id, val_ptr, self._size_src.njmax*1 * sizeof(c_int))
    
    @property
    def efc_rownnz(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_rownnz, dtype=np.int, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_rownnz.setter
    def efc_rownnz(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.efc_rownnz, val_ptr, self._size_src.njmax*1 * sizeof(c_int))
    
    @property
    def efc_rowadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_rowadr, dtype=np.int, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_rowadr.setter
    def efc_rowadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.efc_rowadr, val_ptr, self._size_src.njmax*1 * sizeof(c_int))
    
    @property
    def efc_colind(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_colind, dtype=np.int, count=(self._size_src.njmax*self._size_src.nv)), (self._size_src.njmax, self._size_src.nv, ))
        arr.setflags(write=False)
        return arr
    
    @efc_colind.setter
    def efc_colind(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.efc_colind, val_ptr, self._size_src.njmax*self._size_src.nv * sizeof(c_int))
    
    @property
    def efc_rownnz_T(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_rownnz_T, dtype=np.int, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_rownnz_T.setter
    def efc_rownnz_T(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.efc_rownnz_T, val_ptr, self._size_src.nv*1 * sizeof(c_int))
    
    @property
    def efc_rowadr_T(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_rowadr_T, dtype=np.int, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_rowadr_T.setter
    def efc_rowadr_T(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.efc_rowadr_T, val_ptr, self._size_src.nv*1 * sizeof(c_int))
    
    @property
    def efc_colind_T(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_colind_T, dtype=np.int, count=(self._size_src.nv*self._size_src.njmax)), (self._size_src.nv, self._size_src.njmax, ))
        arr.setflags(write=False)
        return arr
    
    @efc_colind_T.setter
    def efc_colind_T(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.efc_colind_T, val_ptr, self._size_src.nv*self._size_src.njmax * sizeof(c_int))
    
    @property
    def efc_solref(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_solref, dtype=np.double, count=(self._size_src.njmax*2)), (self._size_src.njmax, 2, ))
        arr.setflags(write=False)
        return arr
    
    @efc_solref.setter
    def efc_solref(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_solref, val_ptr, self._size_src.njmax*2 * sizeof(c_double))
    
    @property
    def efc_solimp(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_solimp, dtype=np.double, count=(self._size_src.njmax*3)), (self._size_src.njmax, 3, ))
        arr.setflags(write=False)
        return arr
    
    @efc_solimp.setter
    def efc_solimp(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_solimp, val_ptr, self._size_src.njmax*3 * sizeof(c_double))
    
    @property
    def efc_margin(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_margin, dtype=np.double, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_margin.setter
    def efc_margin(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_margin, val_ptr, self._size_src.njmax*1 * sizeof(c_double))
    
    @property
    def efc_frictionloss(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_frictionloss, dtype=np.double, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_frictionloss.setter
    def efc_frictionloss(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_frictionloss, val_ptr, self._size_src.njmax*1 * sizeof(c_double))
    
    @property
    def efc_pos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_pos, dtype=np.double, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_pos.setter
    def efc_pos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_pos, val_ptr, self._size_src.njmax*1 * sizeof(c_double))
    
    @property
    def efc_J(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_J, dtype=np.double, count=(self._size_src.njmax*self._size_src.nv)), (self._size_src.njmax, self._size_src.nv, ))
        arr.setflags(write=False)
        return arr
    
    @efc_J.setter
    def efc_J(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_J, val_ptr, self._size_src.njmax*self._size_src.nv * sizeof(c_double))
    
    @property
    def efc_J_T(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_J_T, dtype=np.double, count=(self._size_src.nv*self._size_src.njmax)), (self._size_src.nv, self._size_src.njmax, ))
        arr.setflags(write=False)
        return arr
    
    @efc_J_T.setter
    def efc_J_T(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_J_T, val_ptr, self._size_src.nv*self._size_src.njmax * sizeof(c_double))
    
    @property
    def efc_diagApprox(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_diagApprox, dtype=np.double, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_diagApprox.setter
    def efc_diagApprox(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_diagApprox, val_ptr, self._size_src.njmax*1 * sizeof(c_double))
    
    @property
    def efc_D(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_D, dtype=np.double, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_D.setter
    def efc_D(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_D, val_ptr, self._size_src.njmax*1 * sizeof(c_double))
    
    @property
    def efc_R(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_R, dtype=np.double, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_R.setter
    def efc_R(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_R, val_ptr, self._size_src.njmax*1 * sizeof(c_double))
    
    @property
    def efc_AR(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_AR, dtype=np.double, count=(self._size_src.njmax*self._size_src.njmax)), (self._size_src.njmax, self._size_src.njmax, ))
        arr.setflags(write=False)
        return arr
    
    @efc_AR.setter
    def efc_AR(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_AR, val_ptr, self._size_src.njmax*self._size_src.njmax * sizeof(c_double))
    
    @property
    def e_ARchol(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.e_ARchol, dtype=np.double, count=(self._size_src.nemax*self._size_src.nemax)), (self._size_src.nemax, self._size_src.nemax, ))
        arr.setflags(write=False)
        return arr
    
    @e_ARchol.setter
    def e_ARchol(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.e_ARchol, val_ptr, self._size_src.nemax*self._size_src.nemax * sizeof(c_double))
    
    @property
    def fc_e_rect(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.fc_e_rect, dtype=np.double, count=(self._size_src.njmax*self._size_src.nemax)), (self._size_src.njmax, self._size_src.nemax, ))
        arr.setflags(write=False)
        return arr
    
    @fc_e_rect.setter
    def fc_e_rect(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.fc_e_rect, val_ptr, self._size_src.njmax*self._size_src.nemax * sizeof(c_double))
    
    @property
    def fc_AR(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.fc_AR, dtype=np.double, count=(self._size_src.njmax*self._size_src.njmax)), (self._size_src.njmax, self._size_src.njmax, ))
        arr.setflags(write=False)
        return arr
    
    @fc_AR.setter
    def fc_AR(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.fc_AR, val_ptr, self._size_src.njmax*self._size_src.njmax * sizeof(c_double))
    
    @property
    def ten_velocity(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.ten_velocity, dtype=np.double, count=(self._size_src.ntendon*1)), (self._size_src.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @ten_velocity.setter
    def ten_velocity(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.ten_velocity, val_ptr, self._size_src.ntendon*1 * sizeof(c_double))
    
    @property
    def actuator_velocity(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_velocity, dtype=np.double, count=(self._size_src.nu*1)), (self._size_src.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_velocity.setter
    def actuator_velocity(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_velocity, val_ptr, self._size_src.nu*1 * sizeof(c_double))
    
    @property
    def cvel(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cvel, dtype=np.double, count=(self._size_src.nbody*6)), (self._size_src.nbody, 6, ))
        arr.setflags(write=False)
        return arr
    
    @cvel.setter
    def cvel(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cvel, val_ptr, self._size_src.nbody*6 * sizeof(c_double))
    
    @property
    def cdof_dot(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cdof_dot, dtype=np.double, count=(self._size_src.nv*6)), (self._size_src.nv, 6, ))
        arr.setflags(write=False)
        return arr
    
    @cdof_dot.setter
    def cdof_dot(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cdof_dot, val_ptr, self._size_src.nv*6 * sizeof(c_double))
    
    @property
    def qfrc_bias(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qfrc_bias, dtype=np.double, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qfrc_bias.setter
    def qfrc_bias(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qfrc_bias, val_ptr, self._size_src.nv*1 * sizeof(c_double))
    
    @property
    def qfrc_passive(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qfrc_passive, dtype=np.double, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qfrc_passive.setter
    def qfrc_passive(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qfrc_passive, val_ptr, self._size_src.nv*1 * sizeof(c_double))
    
    @property
    def efc_vel(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_vel, dtype=np.double, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_vel.setter
    def efc_vel(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_vel, val_ptr, self._size_src.njmax*1 * sizeof(c_double))
    
    @property
    def efc_aref(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_aref, dtype=np.double, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_aref.setter
    def efc_aref(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_aref, val_ptr, self._size_src.njmax*1 * sizeof(c_double))
    
    @property
    def actuator_force(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_force, dtype=np.double, count=(self._size_src.nu*1)), (self._size_src.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_force.setter
    def actuator_force(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_force, val_ptr, self._size_src.nu*1 * sizeof(c_double))
    
    @property
    def qfrc_actuator(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qfrc_actuator, dtype=np.double, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qfrc_actuator.setter
    def qfrc_actuator(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qfrc_actuator, val_ptr, self._size_src.nv*1 * sizeof(c_double))
    
    @property
    def qfrc_unc(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qfrc_unc, dtype=np.double, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qfrc_unc.setter
    def qfrc_unc(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qfrc_unc, val_ptr, self._size_src.nv*1 * sizeof(c_double))
    
    @property
    def qacc_unc(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qacc_unc, dtype=np.double, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qacc_unc.setter
    def qacc_unc(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qacc_unc, val_ptr, self._size_src.nv*1 * sizeof(c_double))
    
    @property
    def efc_b(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_b, dtype=np.double, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_b.setter
    def efc_b(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_b, val_ptr, self._size_src.njmax*1 * sizeof(c_double))
    
    @property
    def fc_b(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.fc_b, dtype=np.double, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @fc_b.setter
    def fc_b(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.fc_b, val_ptr, self._size_src.njmax*1 * sizeof(c_double))
    
    @property
    def efc_force(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.efc_force, dtype=np.double, count=(self._size_src.njmax*1)), (self._size_src.njmax, 1, ))
        arr.setflags(write=False)
        return arr
    
    @efc_force.setter
    def efc_force(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.efc_force, val_ptr, self._size_src.njmax*1 * sizeof(c_double))
    
    @property
    def qfrc_constraint(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qfrc_constraint, dtype=np.double, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qfrc_constraint.setter
    def qfrc_constraint(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qfrc_constraint, val_ptr, self._size_src.nv*1 * sizeof(c_double))
    
    @property
    def qfrc_inverse(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qfrc_inverse, dtype=np.double, count=(self._size_src.nv*1)), (self._size_src.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qfrc_inverse.setter
    def qfrc_inverse(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qfrc_inverse, val_ptr, self._size_src.nv*1 * sizeof(c_double))
    
    @property
    def cacc(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cacc, dtype=np.double, count=(self._size_src.nbody*6)), (self._size_src.nbody, 6, ))
        arr.setflags(write=False)
        return arr
    
    @cacc.setter
    def cacc(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cacc, val_ptr, self._size_src.nbody*6 * sizeof(c_double))
    
    @property
    def cfrc_int(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cfrc_int, dtype=np.double, count=(self._size_src.nbody*6)), (self._size_src.nbody, 6, ))
        arr.setflags(write=False)
        return arr
    
    @cfrc_int.setter
    def cfrc_int(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cfrc_int, val_ptr, self._size_src.nbody*6 * sizeof(c_double))
    
    @property
    def cfrc_ext(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cfrc_ext, dtype=np.double, count=(self._size_src.nbody*6)), (self._size_src.nbody, 6, ))
        arr.setflags(write=False)
        return arr
    
    @cfrc_ext.setter
    def cfrc_ext(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cfrc_ext, val_ptr, self._size_src.nbody*6 * sizeof(c_double))

class MjModelWrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    
    @property
    def nq(self):
        return self._wrapped.contents.nq
    
    @nq.setter
    def nq(self, value):
        self._wrapped.contents.nq = value
    
    @property
    def nv(self):
        return self._wrapped.contents.nv
    
    @nv.setter
    def nv(self, value):
        self._wrapped.contents.nv = value
    
    @property
    def nu(self):
        return self._wrapped.contents.nu
    
    @nu.setter
    def nu(self, value):
        self._wrapped.contents.nu = value
    
    @property
    def na(self):
        return self._wrapped.contents.na
    
    @na.setter
    def na(self, value):
        self._wrapped.contents.na = value
    
    @property
    def nbody(self):
        return self._wrapped.contents.nbody
    
    @nbody.setter
    def nbody(self, value):
        self._wrapped.contents.nbody = value
    
    @property
    def njnt(self):
        return self._wrapped.contents.njnt
    
    @njnt.setter
    def njnt(self, value):
        self._wrapped.contents.njnt = value
    
    @property
    def ngeom(self):
        return self._wrapped.contents.ngeom
    
    @ngeom.setter
    def ngeom(self, value):
        self._wrapped.contents.ngeom = value
    
    @property
    def nsite(self):
        return self._wrapped.contents.nsite
    
    @nsite.setter
    def nsite(self, value):
        self._wrapped.contents.nsite = value
    
    @property
    def ncam(self):
        return self._wrapped.contents.ncam
    
    @ncam.setter
    def ncam(self, value):
        self._wrapped.contents.ncam = value
    
    @property
    def nlight(self):
        return self._wrapped.contents.nlight
    
    @nlight.setter
    def nlight(self, value):
        self._wrapped.contents.nlight = value
    
    @property
    def nmesh(self):
        return self._wrapped.contents.nmesh
    
    @nmesh.setter
    def nmesh(self, value):
        self._wrapped.contents.nmesh = value
    
    @property
    def nmeshvert(self):
        return self._wrapped.contents.nmeshvert
    
    @nmeshvert.setter
    def nmeshvert(self, value):
        self._wrapped.contents.nmeshvert = value
    
    @property
    def nmeshface(self):
        return self._wrapped.contents.nmeshface
    
    @nmeshface.setter
    def nmeshface(self, value):
        self._wrapped.contents.nmeshface = value
    
    @property
    def nmeshgraph(self):
        return self._wrapped.contents.nmeshgraph
    
    @nmeshgraph.setter
    def nmeshgraph(self, value):
        self._wrapped.contents.nmeshgraph = value
    
    @property
    def nhfield(self):
        return self._wrapped.contents.nhfield
    
    @nhfield.setter
    def nhfield(self, value):
        self._wrapped.contents.nhfield = value
    
    @property
    def nhfielddata(self):
        return self._wrapped.contents.nhfielddata
    
    @nhfielddata.setter
    def nhfielddata(self, value):
        self._wrapped.contents.nhfielddata = value
    
    @property
    def ntex(self):
        return self._wrapped.contents.ntex
    
    @ntex.setter
    def ntex(self, value):
        self._wrapped.contents.ntex = value
    
    @property
    def ntexdata(self):
        return self._wrapped.contents.ntexdata
    
    @ntexdata.setter
    def ntexdata(self, value):
        self._wrapped.contents.ntexdata = value
    
    @property
    def nmat(self):
        return self._wrapped.contents.nmat
    
    @nmat.setter
    def nmat(self, value):
        self._wrapped.contents.nmat = value
    
    @property
    def npair(self):
        return self._wrapped.contents.npair
    
    @npair.setter
    def npair(self, value):
        self._wrapped.contents.npair = value
    
    @property
    def nexclude(self):
        return self._wrapped.contents.nexclude
    
    @nexclude.setter
    def nexclude(self, value):
        self._wrapped.contents.nexclude = value
    
    @property
    def neq(self):
        return self._wrapped.contents.neq
    
    @neq.setter
    def neq(self, value):
        self._wrapped.contents.neq = value
    
    @property
    def ntendon(self):
        return self._wrapped.contents.ntendon
    
    @ntendon.setter
    def ntendon(self, value):
        self._wrapped.contents.ntendon = value
    
    @property
    def nwrap(self):
        return self._wrapped.contents.nwrap
    
    @nwrap.setter
    def nwrap(self, value):
        self._wrapped.contents.nwrap = value
    
    @property
    def nsensor(self):
        return self._wrapped.contents.nsensor
    
    @nsensor.setter
    def nsensor(self, value):
        self._wrapped.contents.nsensor = value
    
    @property
    def nnumeric(self):
        return self._wrapped.contents.nnumeric
    
    @nnumeric.setter
    def nnumeric(self, value):
        self._wrapped.contents.nnumeric = value
    
    @property
    def nnumericdata(self):
        return self._wrapped.contents.nnumericdata
    
    @nnumericdata.setter
    def nnumericdata(self, value):
        self._wrapped.contents.nnumericdata = value
    
    @property
    def ntext(self):
        return self._wrapped.contents.ntext
    
    @ntext.setter
    def ntext(self, value):
        self._wrapped.contents.ntext = value
    
    @property
    def ntextdata(self):
        return self._wrapped.contents.ntextdata
    
    @ntextdata.setter
    def ntextdata(self, value):
        self._wrapped.contents.ntextdata = value
    
    @property
    def nkey(self):
        return self._wrapped.contents.nkey
    
    @nkey.setter
    def nkey(self, value):
        self._wrapped.contents.nkey = value
    
    @property
    def nuser_body(self):
        return self._wrapped.contents.nuser_body
    
    @nuser_body.setter
    def nuser_body(self, value):
        self._wrapped.contents.nuser_body = value
    
    @property
    def nuser_jnt(self):
        return self._wrapped.contents.nuser_jnt
    
    @nuser_jnt.setter
    def nuser_jnt(self, value):
        self._wrapped.contents.nuser_jnt = value
    
    @property
    def nuser_geom(self):
        return self._wrapped.contents.nuser_geom
    
    @nuser_geom.setter
    def nuser_geom(self, value):
        self._wrapped.contents.nuser_geom = value
    
    @property
    def nuser_site(self):
        return self._wrapped.contents.nuser_site
    
    @nuser_site.setter
    def nuser_site(self, value):
        self._wrapped.contents.nuser_site = value
    
    @property
    def nuser_tendon(self):
        return self._wrapped.contents.nuser_tendon
    
    @nuser_tendon.setter
    def nuser_tendon(self, value):
        self._wrapped.contents.nuser_tendon = value
    
    @property
    def nuser_actuator(self):
        return self._wrapped.contents.nuser_actuator
    
    @nuser_actuator.setter
    def nuser_actuator(self, value):
        self._wrapped.contents.nuser_actuator = value
    
    @property
    def nuser_sensor(self):
        return self._wrapped.contents.nuser_sensor
    
    @nuser_sensor.setter
    def nuser_sensor(self, value):
        self._wrapped.contents.nuser_sensor = value
    
    @property
    def nnames(self):
        return self._wrapped.contents.nnames
    
    @nnames.setter
    def nnames(self, value):
        self._wrapped.contents.nnames = value
    
    @property
    def nM(self):
        return self._wrapped.contents.nM
    
    @nM.setter
    def nM(self, value):
        self._wrapped.contents.nM = value
    
    @property
    def nemax(self):
        return self._wrapped.contents.nemax
    
    @nemax.setter
    def nemax(self, value):
        self._wrapped.contents.nemax = value
    
    @property
    def njmax(self):
        return self._wrapped.contents.njmax
    
    @njmax.setter
    def njmax(self, value):
        self._wrapped.contents.njmax = value
    
    @property
    def nconmax(self):
        return self._wrapped.contents.nconmax
    
    @nconmax.setter
    def nconmax(self, value):
        self._wrapped.contents.nconmax = value
    
    @property
    def nstack(self):
        return self._wrapped.contents.nstack
    
    @nstack.setter
    def nstack(self, value):
        self._wrapped.contents.nstack = value
    
    @property
    def nuserdata(self):
        return self._wrapped.contents.nuserdata
    
    @nuserdata.setter
    def nuserdata(self, value):
        self._wrapped.contents.nuserdata = value
    
    @property
    def nmocap(self):
        return self._wrapped.contents.nmocap
    
    @nmocap.setter
    def nmocap(self, value):
        self._wrapped.contents.nmocap = value
    
    @property
    def nsensordata(self):
        return self._wrapped.contents.nsensordata
    
    @nsensordata.setter
    def nsensordata(self, value):
        self._wrapped.contents.nsensordata = value
    
    @property
    def nbuffer(self):
        return self._wrapped.contents.nbuffer
    
    @nbuffer.setter
    def nbuffer(self, value):
        self._wrapped.contents.nbuffer = value
    
    @property
    def opt(self):
        return self._wrapped.contents.opt
    
    @opt.setter
    def opt(self, value):
        self._wrapped.contents.opt = value
    
    @property
    def vis(self):
        return self._wrapped.contents.vis
    
    @vis.setter
    def vis(self, value):
        self._wrapped.contents.vis = value
    
    @property
    def stat(self):
        return self._wrapped.contents.stat
    
    @stat.setter
    def stat(self, value):
        self._wrapped.contents.stat = value
    
    @property
    def buffer(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.buffer, dtype=np.uint8, count=(self.nbuffer)), (self.nbuffer, ))
        arr.setflags(write=False)
        return arr
    
    @buffer.setter
    def buffer(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.buffer, val_ptr, self.nbuffer * sizeof(c_ubyte))
    
    @property
    def qpos0(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qpos0, dtype=np.double, count=(self.nq*1)), (self.nq, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qpos0.setter
    def qpos0(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qpos0, val_ptr, self.nq*1 * sizeof(c_double))
    
    @property
    def qpos_spring(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.qpos_spring, dtype=np.double, count=(self.nq*1)), (self.nq, 1, ))
        arr.setflags(write=False)
        return arr
    
    @qpos_spring.setter
    def qpos_spring(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.qpos_spring, val_ptr, self.nq*1 * sizeof(c_double))
    
    @property
    def body_parentid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_parentid, dtype=np.int, count=(self.nbody*1)), (self.nbody, 1, ))
        arr.setflags(write=False)
        return arr
    
    @body_parentid.setter
    def body_parentid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.body_parentid, val_ptr, self.nbody*1 * sizeof(c_int))
    
    @property
    def body_rootid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_rootid, dtype=np.int, count=(self.nbody*1)), (self.nbody, 1, ))
        arr.setflags(write=False)
        return arr
    
    @body_rootid.setter
    def body_rootid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.body_rootid, val_ptr, self.nbody*1 * sizeof(c_int))
    
    @property
    def body_weldid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_weldid, dtype=np.int, count=(self.nbody*1)), (self.nbody, 1, ))
        arr.setflags(write=False)
        return arr
    
    @body_weldid.setter
    def body_weldid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.body_weldid, val_ptr, self.nbody*1 * sizeof(c_int))
    
    @property
    def body_mocapid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_mocapid, dtype=np.int, count=(self.nbody*1)), (self.nbody, 1, ))
        arr.setflags(write=False)
        return arr
    
    @body_mocapid.setter
    def body_mocapid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.body_mocapid, val_ptr, self.nbody*1 * sizeof(c_int))
    
    @property
    def body_jntnum(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_jntnum, dtype=np.int, count=(self.nbody*1)), (self.nbody, 1, ))
        arr.setflags(write=False)
        return arr
    
    @body_jntnum.setter
    def body_jntnum(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.body_jntnum, val_ptr, self.nbody*1 * sizeof(c_int))
    
    @property
    def body_jntadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_jntadr, dtype=np.int, count=(self.nbody*1)), (self.nbody, 1, ))
        arr.setflags(write=False)
        return arr
    
    @body_jntadr.setter
    def body_jntadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.body_jntadr, val_ptr, self.nbody*1 * sizeof(c_int))
    
    @property
    def body_dofnum(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_dofnum, dtype=np.int, count=(self.nbody*1)), (self.nbody, 1, ))
        arr.setflags(write=False)
        return arr
    
    @body_dofnum.setter
    def body_dofnum(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.body_dofnum, val_ptr, self.nbody*1 * sizeof(c_int))
    
    @property
    def body_dofadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_dofadr, dtype=np.int, count=(self.nbody*1)), (self.nbody, 1, ))
        arr.setflags(write=False)
        return arr
    
    @body_dofadr.setter
    def body_dofadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.body_dofadr, val_ptr, self.nbody*1 * sizeof(c_int))
    
    @property
    def body_geomnum(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_geomnum, dtype=np.int, count=(self.nbody*1)), (self.nbody, 1, ))
        arr.setflags(write=False)
        return arr
    
    @body_geomnum.setter
    def body_geomnum(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.body_geomnum, val_ptr, self.nbody*1 * sizeof(c_int))
    
    @property
    def body_geomadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_geomadr, dtype=np.int, count=(self.nbody*1)), (self.nbody, 1, ))
        arr.setflags(write=False)
        return arr
    
    @body_geomadr.setter
    def body_geomadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.body_geomadr, val_ptr, self.nbody*1 * sizeof(c_int))
    
    @property
    def body_pos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_pos, dtype=np.double, count=(self.nbody*3)), (self.nbody, 3, ))
        arr.setflags(write=False)
        return arr
    
    @body_pos.setter
    def body_pos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.body_pos, val_ptr, self.nbody*3 * sizeof(c_double))
    
    @property
    def body_quat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_quat, dtype=np.double, count=(self.nbody*4)), (self.nbody, 4, ))
        arr.setflags(write=False)
        return arr
    
    @body_quat.setter
    def body_quat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.body_quat, val_ptr, self.nbody*4 * sizeof(c_double))
    
    @property
    def body_ipos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_ipos, dtype=np.double, count=(self.nbody*3)), (self.nbody, 3, ))
        arr.setflags(write=False)
        return arr
    
    @body_ipos.setter
    def body_ipos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.body_ipos, val_ptr, self.nbody*3 * sizeof(c_double))
    
    @property
    def body_iquat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_iquat, dtype=np.double, count=(self.nbody*4)), (self.nbody, 4, ))
        arr.setflags(write=False)
        return arr
    
    @body_iquat.setter
    def body_iquat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.body_iquat, val_ptr, self.nbody*4 * sizeof(c_double))
    
    @property
    def body_mass(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_mass, dtype=np.double, count=(self.nbody*1)), (self.nbody, 1, ))
        arr.setflags(write=False)
        return arr
    
    @body_mass.setter
    def body_mass(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.body_mass, val_ptr, self.nbody*1 * sizeof(c_double))
    
    @property
    def body_inertia(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_inertia, dtype=np.double, count=(self.nbody*3)), (self.nbody, 3, ))
        arr.setflags(write=False)
        return arr
    
    @body_inertia.setter
    def body_inertia(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.body_inertia, val_ptr, self.nbody*3 * sizeof(c_double))
    
    @property
    def body_invweight0(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_invweight0, dtype=np.double, count=(self.nbody*2)), (self.nbody, 2, ))
        arr.setflags(write=False)
        return arr
    
    @body_invweight0.setter
    def body_invweight0(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.body_invweight0, val_ptr, self.nbody*2 * sizeof(c_double))
    
    @property
    def body_user(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.body_user, dtype=np.double, count=(self.nbody*self.nuser_body)), (self.nbody, self.nuser_body, ))
        arr.setflags(write=False)
        return arr
    
    @body_user.setter
    def body_user(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.body_user, val_ptr, self.nbody*self.nuser_body * sizeof(c_double))
    
    @property
    def jnt_type(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.jnt_type, dtype=np.int, count=(self.njnt*1)), (self.njnt, 1, ))
        arr.setflags(write=False)
        return arr
    
    @jnt_type.setter
    def jnt_type(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.jnt_type, val_ptr, self.njnt*1 * sizeof(c_int))
    
    @property
    def jnt_qposadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.jnt_qposadr, dtype=np.int, count=(self.njnt*1)), (self.njnt, 1, ))
        arr.setflags(write=False)
        return arr
    
    @jnt_qposadr.setter
    def jnt_qposadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.jnt_qposadr, val_ptr, self.njnt*1 * sizeof(c_int))
    
    @property
    def jnt_dofadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.jnt_dofadr, dtype=np.int, count=(self.njnt*1)), (self.njnt, 1, ))
        arr.setflags(write=False)
        return arr
    
    @jnt_dofadr.setter
    def jnt_dofadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.jnt_dofadr, val_ptr, self.njnt*1 * sizeof(c_int))
    
    @property
    def jnt_bodyid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.jnt_bodyid, dtype=np.int, count=(self.njnt*1)), (self.njnt, 1, ))
        arr.setflags(write=False)
        return arr
    
    @jnt_bodyid.setter
    def jnt_bodyid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.jnt_bodyid, val_ptr, self.njnt*1 * sizeof(c_int))
    
    @property
    def jnt_limited(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.jnt_limited, dtype=np.uint8, count=(self.njnt*1)), (self.njnt, 1, ))
        arr.setflags(write=False)
        return arr
    
    @jnt_limited.setter
    def jnt_limited(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.jnt_limited, val_ptr, self.njnt*1 * sizeof(c_ubyte))
    
    @property
    def jnt_solref(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.jnt_solref, dtype=np.double, count=(self.njnt*2)), (self.njnt, 2, ))
        arr.setflags(write=False)
        return arr
    
    @jnt_solref.setter
    def jnt_solref(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.jnt_solref, val_ptr, self.njnt*2 * sizeof(c_double))
    
    @property
    def jnt_solimp(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.jnt_solimp, dtype=np.double, count=(self.njnt*3)), (self.njnt, 3, ))
        arr.setflags(write=False)
        return arr
    
    @jnt_solimp.setter
    def jnt_solimp(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.jnt_solimp, val_ptr, self.njnt*3 * sizeof(c_double))
    
    @property
    def jnt_pos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.jnt_pos, dtype=np.double, count=(self.njnt*3)), (self.njnt, 3, ))
        arr.setflags(write=False)
        return arr
    
    @jnt_pos.setter
    def jnt_pos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.jnt_pos, val_ptr, self.njnt*3 * sizeof(c_double))
    
    @property
    def jnt_axis(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.jnt_axis, dtype=np.double, count=(self.njnt*3)), (self.njnt, 3, ))
        arr.setflags(write=False)
        return arr
    
    @jnt_axis.setter
    def jnt_axis(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.jnt_axis, val_ptr, self.njnt*3 * sizeof(c_double))
    
    @property
    def jnt_stiffness(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.jnt_stiffness, dtype=np.double, count=(self.njnt*1)), (self.njnt, 1, ))
        arr.setflags(write=False)
        return arr
    
    @jnt_stiffness.setter
    def jnt_stiffness(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.jnt_stiffness, val_ptr, self.njnt*1 * sizeof(c_double))
    
    @property
    def jnt_range(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.jnt_range, dtype=np.double, count=(self.njnt*2)), (self.njnt, 2, ))
        arr.setflags(write=False)
        return arr
    
    @jnt_range.setter
    def jnt_range(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.jnt_range, val_ptr, self.njnt*2 * sizeof(c_double))
    
    @property
    def jnt_margin(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.jnt_margin, dtype=np.double, count=(self.njnt*1)), (self.njnt, 1, ))
        arr.setflags(write=False)
        return arr
    
    @jnt_margin.setter
    def jnt_margin(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.jnt_margin, val_ptr, self.njnt*1 * sizeof(c_double))
    
    @property
    def jnt_user(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.jnt_user, dtype=np.double, count=(self.njnt*self.nuser_jnt)), (self.njnt, self.nuser_jnt, ))
        arr.setflags(write=False)
        return arr
    
    @jnt_user.setter
    def jnt_user(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.jnt_user, val_ptr, self.njnt*self.nuser_jnt * sizeof(c_double))
    
    @property
    def dof_bodyid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.dof_bodyid, dtype=np.int, count=(self.nv*1)), (self.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @dof_bodyid.setter
    def dof_bodyid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.dof_bodyid, val_ptr, self.nv*1 * sizeof(c_int))
    
    @property
    def dof_jntid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.dof_jntid, dtype=np.int, count=(self.nv*1)), (self.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @dof_jntid.setter
    def dof_jntid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.dof_jntid, val_ptr, self.nv*1 * sizeof(c_int))
    
    @property
    def dof_parentid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.dof_parentid, dtype=np.int, count=(self.nv*1)), (self.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @dof_parentid.setter
    def dof_parentid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.dof_parentid, val_ptr, self.nv*1 * sizeof(c_int))
    
    @property
    def dof_Madr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.dof_Madr, dtype=np.int, count=(self.nv*1)), (self.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @dof_Madr.setter
    def dof_Madr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.dof_Madr, val_ptr, self.nv*1 * sizeof(c_int))
    
    @property
    def dof_frictional(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.dof_frictional, dtype=np.uint8, count=(self.nv*1)), (self.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @dof_frictional.setter
    def dof_frictional(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.dof_frictional, val_ptr, self.nv*1 * sizeof(c_ubyte))
    
    @property
    def dof_solref(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.dof_solref, dtype=np.double, count=(self.nv*2)), (self.nv, 2, ))
        arr.setflags(write=False)
        return arr
    
    @dof_solref.setter
    def dof_solref(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.dof_solref, val_ptr, self.nv*2 * sizeof(c_double))
    
    @property
    def dof_solimp(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.dof_solimp, dtype=np.double, count=(self.nv*3)), (self.nv, 3, ))
        arr.setflags(write=False)
        return arr
    
    @dof_solimp.setter
    def dof_solimp(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.dof_solimp, val_ptr, self.nv*3 * sizeof(c_double))
    
    @property
    def dof_frictionloss(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.dof_frictionloss, dtype=np.double, count=(self.nv*1)), (self.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @dof_frictionloss.setter
    def dof_frictionloss(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.dof_frictionloss, val_ptr, self.nv*1 * sizeof(c_double))
    
    @property
    def dof_armature(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.dof_armature, dtype=np.double, count=(self.nv*1)), (self.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @dof_armature.setter
    def dof_armature(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.dof_armature, val_ptr, self.nv*1 * sizeof(c_double))
    
    @property
    def dof_damping(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.dof_damping, dtype=np.double, count=(self.nv*1)), (self.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @dof_damping.setter
    def dof_damping(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.dof_damping, val_ptr, self.nv*1 * sizeof(c_double))
    
    @property
    def dof_invweight0(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.dof_invweight0, dtype=np.double, count=(self.nv*1)), (self.nv, 1, ))
        arr.setflags(write=False)
        return arr
    
    @dof_invweight0.setter
    def dof_invweight0(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.dof_invweight0, val_ptr, self.nv*1 * sizeof(c_double))
    
    @property
    def geom_type(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_type, dtype=np.int, count=(self.ngeom*1)), (self.ngeom, 1, ))
        arr.setflags(write=False)
        return arr
    
    @geom_type.setter
    def geom_type(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.geom_type, val_ptr, self.ngeom*1 * sizeof(c_int))
    
    @property
    def geom_contype(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_contype, dtype=np.int, count=(self.ngeom*1)), (self.ngeom, 1, ))
        arr.setflags(write=False)
        return arr
    
    @geom_contype.setter
    def geom_contype(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.geom_contype, val_ptr, self.ngeom*1 * sizeof(c_int))
    
    @property
    def geom_conaffinity(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_conaffinity, dtype=np.int, count=(self.ngeom*1)), (self.ngeom, 1, ))
        arr.setflags(write=False)
        return arr
    
    @geom_conaffinity.setter
    def geom_conaffinity(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.geom_conaffinity, val_ptr, self.ngeom*1 * sizeof(c_int))
    
    @property
    def geom_condim(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_condim, dtype=np.int, count=(self.ngeom*1)), (self.ngeom, 1, ))
        arr.setflags(write=False)
        return arr
    
    @geom_condim.setter
    def geom_condim(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.geom_condim, val_ptr, self.ngeom*1 * sizeof(c_int))
    
    @property
    def geom_bodyid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_bodyid, dtype=np.int, count=(self.ngeom*1)), (self.ngeom, 1, ))
        arr.setflags(write=False)
        return arr
    
    @geom_bodyid.setter
    def geom_bodyid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.geom_bodyid, val_ptr, self.ngeom*1 * sizeof(c_int))
    
    @property
    def geom_dataid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_dataid, dtype=np.int, count=(self.ngeom*1)), (self.ngeom, 1, ))
        arr.setflags(write=False)
        return arr
    
    @geom_dataid.setter
    def geom_dataid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.geom_dataid, val_ptr, self.ngeom*1 * sizeof(c_int))
    
    @property
    def geom_matid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_matid, dtype=np.int, count=(self.ngeom*1)), (self.ngeom, 1, ))
        arr.setflags(write=False)
        return arr
    
    @geom_matid.setter
    def geom_matid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.geom_matid, val_ptr, self.ngeom*1 * sizeof(c_int))
    
    @property
    def geom_group(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_group, dtype=np.int, count=(self.ngeom*1)), (self.ngeom, 1, ))
        arr.setflags(write=False)
        return arr
    
    @geom_group.setter
    def geom_group(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.geom_group, val_ptr, self.ngeom*1 * sizeof(c_int))
    
    @property
    def geom_solmix(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_solmix, dtype=np.double, count=(self.ngeom*1)), (self.ngeom, 1, ))
        arr.setflags(write=False)
        return arr
    
    @geom_solmix.setter
    def geom_solmix(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.geom_solmix, val_ptr, self.ngeom*1 * sizeof(c_double))
    
    @property
    def geom_solref(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_solref, dtype=np.double, count=(self.ngeom*2)), (self.ngeom, 2, ))
        arr.setflags(write=False)
        return arr
    
    @geom_solref.setter
    def geom_solref(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.geom_solref, val_ptr, self.ngeom*2 * sizeof(c_double))
    
    @property
    def geom_solimp(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_solimp, dtype=np.double, count=(self.ngeom*3)), (self.ngeom, 3, ))
        arr.setflags(write=False)
        return arr
    
    @geom_solimp.setter
    def geom_solimp(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.geom_solimp, val_ptr, self.ngeom*3 * sizeof(c_double))
    
    @property
    def geom_size(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_size, dtype=np.double, count=(self.ngeom*3)), (self.ngeom, 3, ))
        arr.setflags(write=False)
        return arr
    
    @geom_size.setter
    def geom_size(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.geom_size, val_ptr, self.ngeom*3 * sizeof(c_double))
    
    @property
    def geom_rbound(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_rbound, dtype=np.double, count=(self.ngeom*1)), (self.ngeom, 1, ))
        arr.setflags(write=False)
        return arr
    
    @geom_rbound.setter
    def geom_rbound(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.geom_rbound, val_ptr, self.ngeom*1 * sizeof(c_double))
    
    @property
    def geom_pos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_pos, dtype=np.double, count=(self.ngeom*3)), (self.ngeom, 3, ))
        arr.setflags(write=False)
        return arr
    
    @geom_pos.setter
    def geom_pos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.geom_pos, val_ptr, self.ngeom*3 * sizeof(c_double))
    
    @property
    def geom_quat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_quat, dtype=np.double, count=(self.ngeom*4)), (self.ngeom, 4, ))
        arr.setflags(write=False)
        return arr
    
    @geom_quat.setter
    def geom_quat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.geom_quat, val_ptr, self.ngeom*4 * sizeof(c_double))
    
    @property
    def geom_friction(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_friction, dtype=np.double, count=(self.ngeom*3)), (self.ngeom, 3, ))
        arr.setflags(write=False)
        return arr
    
    @geom_friction.setter
    def geom_friction(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.geom_friction, val_ptr, self.ngeom*3 * sizeof(c_double))
    
    @property
    def geom_margin(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_margin, dtype=np.double, count=(self.ngeom*1)), (self.ngeom, 1, ))
        arr.setflags(write=False)
        return arr
    
    @geom_margin.setter
    def geom_margin(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.geom_margin, val_ptr, self.ngeom*1 * sizeof(c_double))
    
    @property
    def geom_gap(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_gap, dtype=np.double, count=(self.ngeom*1)), (self.ngeom, 1, ))
        arr.setflags(write=False)
        return arr
    
    @geom_gap.setter
    def geom_gap(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.geom_gap, val_ptr, self.ngeom*1 * sizeof(c_double))
    
    @property
    def geom_user(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_user, dtype=np.double, count=(self.ngeom*self.nuser_geom)), (self.ngeom, self.nuser_geom, ))
        arr.setflags(write=False)
        return arr
    
    @geom_user.setter
    def geom_user(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.geom_user, val_ptr, self.ngeom*self.nuser_geom * sizeof(c_double))
    
    @property
    def geom_rgba(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.geom_rgba, dtype=np.float, count=(self.ngeom*4)), (self.ngeom, 4, ))
        arr.setflags(write=False)
        return arr
    
    @geom_rgba.setter
    def geom_rgba(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.geom_rgba, val_ptr, self.ngeom*4 * sizeof(c_float))
    
    @property
    def site_type(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.site_type, dtype=np.int, count=(self.nsite*1)), (self.nsite, 1, ))
        arr.setflags(write=False)
        return arr
    
    @site_type.setter
    def site_type(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.site_type, val_ptr, self.nsite*1 * sizeof(c_int))
    
    @property
    def site_bodyid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.site_bodyid, dtype=np.int, count=(self.nsite*1)), (self.nsite, 1, ))
        arr.setflags(write=False)
        return arr
    
    @site_bodyid.setter
    def site_bodyid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.site_bodyid, val_ptr, self.nsite*1 * sizeof(c_int))
    
    @property
    def site_matid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.site_matid, dtype=np.int, count=(self.nsite*1)), (self.nsite, 1, ))
        arr.setflags(write=False)
        return arr
    
    @site_matid.setter
    def site_matid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.site_matid, val_ptr, self.nsite*1 * sizeof(c_int))
    
    @property
    def site_group(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.site_group, dtype=np.int, count=(self.nsite*1)), (self.nsite, 1, ))
        arr.setflags(write=False)
        return arr
    
    @site_group.setter
    def site_group(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.site_group, val_ptr, self.nsite*1 * sizeof(c_int))
    
    @property
    def site_size(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.site_size, dtype=np.double, count=(self.nsite*3)), (self.nsite, 3, ))
        arr.setflags(write=False)
        return arr
    
    @site_size.setter
    def site_size(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.site_size, val_ptr, self.nsite*3 * sizeof(c_double))
    
    @property
    def site_pos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.site_pos, dtype=np.double, count=(self.nsite*3)), (self.nsite, 3, ))
        arr.setflags(write=False)
        return arr
    
    @site_pos.setter
    def site_pos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.site_pos, val_ptr, self.nsite*3 * sizeof(c_double))
    
    @property
    def site_quat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.site_quat, dtype=np.double, count=(self.nsite*4)), (self.nsite, 4, ))
        arr.setflags(write=False)
        return arr
    
    @site_quat.setter
    def site_quat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.site_quat, val_ptr, self.nsite*4 * sizeof(c_double))
    
    @property
    def site_user(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.site_user, dtype=np.double, count=(self.nsite*self.nuser_site)), (self.nsite, self.nuser_site, ))
        arr.setflags(write=False)
        return arr
    
    @site_user.setter
    def site_user(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.site_user, val_ptr, self.nsite*self.nuser_site * sizeof(c_double))
    
    @property
    def site_rgba(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.site_rgba, dtype=np.float, count=(self.nsite*4)), (self.nsite, 4, ))
        arr.setflags(write=False)
        return arr
    
    @site_rgba.setter
    def site_rgba(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.site_rgba, val_ptr, self.nsite*4 * sizeof(c_float))
    
    @property
    def cam_mode(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cam_mode, dtype=np.int, count=(self.ncam*1)), (self.ncam, 1, ))
        arr.setflags(write=False)
        return arr
    
    @cam_mode.setter
    def cam_mode(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.cam_mode, val_ptr, self.ncam*1 * sizeof(c_int))
    
    @property
    def cam_bodyid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cam_bodyid, dtype=np.int, count=(self.ncam*1)), (self.ncam, 1, ))
        arr.setflags(write=False)
        return arr
    
    @cam_bodyid.setter
    def cam_bodyid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.cam_bodyid, val_ptr, self.ncam*1 * sizeof(c_int))
    
    @property
    def cam_targetbodyid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cam_targetbodyid, dtype=np.int, count=(self.ncam*1)), (self.ncam, 1, ))
        arr.setflags(write=False)
        return arr
    
    @cam_targetbodyid.setter
    def cam_targetbodyid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.cam_targetbodyid, val_ptr, self.ncam*1 * sizeof(c_int))
    
    @property
    def cam_pos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cam_pos, dtype=np.double, count=(self.ncam*3)), (self.ncam, 3, ))
        arr.setflags(write=False)
        return arr
    
    @cam_pos.setter
    def cam_pos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cam_pos, val_ptr, self.ncam*3 * sizeof(c_double))
    
    @property
    def cam_quat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cam_quat, dtype=np.double, count=(self.ncam*4)), (self.ncam, 4, ))
        arr.setflags(write=False)
        return arr
    
    @cam_quat.setter
    def cam_quat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cam_quat, val_ptr, self.ncam*4 * sizeof(c_double))
    
    @property
    def cam_poscom0(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cam_poscom0, dtype=np.double, count=(self.ncam*3)), (self.ncam, 3, ))
        arr.setflags(write=False)
        return arr
    
    @cam_poscom0.setter
    def cam_poscom0(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cam_poscom0, val_ptr, self.ncam*3 * sizeof(c_double))
    
    @property
    def cam_pos0(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cam_pos0, dtype=np.double, count=(self.ncam*3)), (self.ncam, 3, ))
        arr.setflags(write=False)
        return arr
    
    @cam_pos0.setter
    def cam_pos0(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cam_pos0, val_ptr, self.ncam*3 * sizeof(c_double))
    
    @property
    def cam_mat0(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cam_mat0, dtype=np.double, count=(self.ncam*9)), (self.ncam, 9, ))
        arr.setflags(write=False)
        return arr
    
    @cam_mat0.setter
    def cam_mat0(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cam_mat0, val_ptr, self.ncam*9 * sizeof(c_double))
    
    @property
    def cam_fovy(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cam_fovy, dtype=np.double, count=(self.ncam*1)), (self.ncam, 1, ))
        arr.setflags(write=False)
        return arr
    
    @cam_fovy.setter
    def cam_fovy(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cam_fovy, val_ptr, self.ncam*1 * sizeof(c_double))
    
    @property
    def cam_ipd(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.cam_ipd, dtype=np.double, count=(self.ncam*1)), (self.ncam, 1, ))
        arr.setflags(write=False)
        return arr
    
    @cam_ipd.setter
    def cam_ipd(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.cam_ipd, val_ptr, self.ncam*1 * sizeof(c_double))
    
    @property
    def light_mode(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_mode, dtype=np.int, count=(self.nlight*1)), (self.nlight, 1, ))
        arr.setflags(write=False)
        return arr
    
    @light_mode.setter
    def light_mode(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.light_mode, val_ptr, self.nlight*1 * sizeof(c_int))
    
    @property
    def light_bodyid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_bodyid, dtype=np.int, count=(self.nlight*1)), (self.nlight, 1, ))
        arr.setflags(write=False)
        return arr
    
    @light_bodyid.setter
    def light_bodyid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.light_bodyid, val_ptr, self.nlight*1 * sizeof(c_int))
    
    @property
    def light_targetbodyid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_targetbodyid, dtype=np.int, count=(self.nlight*1)), (self.nlight, 1, ))
        arr.setflags(write=False)
        return arr
    
    @light_targetbodyid.setter
    def light_targetbodyid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.light_targetbodyid, val_ptr, self.nlight*1 * sizeof(c_int))
    
    @property
    def light_directional(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_directional, dtype=np.uint8, count=(self.nlight*1)), (self.nlight, 1, ))
        arr.setflags(write=False)
        return arr
    
    @light_directional.setter
    def light_directional(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.light_directional, val_ptr, self.nlight*1 * sizeof(c_ubyte))
    
    @property
    def light_castshadow(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_castshadow, dtype=np.uint8, count=(self.nlight*1)), (self.nlight, 1, ))
        arr.setflags(write=False)
        return arr
    
    @light_castshadow.setter
    def light_castshadow(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.light_castshadow, val_ptr, self.nlight*1 * sizeof(c_ubyte))
    
    @property
    def light_active(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_active, dtype=np.uint8, count=(self.nlight*1)), (self.nlight, 1, ))
        arr.setflags(write=False)
        return arr
    
    @light_active.setter
    def light_active(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.light_active, val_ptr, self.nlight*1 * sizeof(c_ubyte))
    
    @property
    def light_pos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_pos, dtype=np.double, count=(self.nlight*3)), (self.nlight, 3, ))
        arr.setflags(write=False)
        return arr
    
    @light_pos.setter
    def light_pos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.light_pos, val_ptr, self.nlight*3 * sizeof(c_double))
    
    @property
    def light_dir(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_dir, dtype=np.double, count=(self.nlight*3)), (self.nlight, 3, ))
        arr.setflags(write=False)
        return arr
    
    @light_dir.setter
    def light_dir(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.light_dir, val_ptr, self.nlight*3 * sizeof(c_double))
    
    @property
    def light_poscom0(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_poscom0, dtype=np.double, count=(self.nlight*3)), (self.nlight, 3, ))
        arr.setflags(write=False)
        return arr
    
    @light_poscom0.setter
    def light_poscom0(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.light_poscom0, val_ptr, self.nlight*3 * sizeof(c_double))
    
    @property
    def light_pos0(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_pos0, dtype=np.double, count=(self.nlight*3)), (self.nlight, 3, ))
        arr.setflags(write=False)
        return arr
    
    @light_pos0.setter
    def light_pos0(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.light_pos0, val_ptr, self.nlight*3 * sizeof(c_double))
    
    @property
    def light_dir0(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_dir0, dtype=np.double, count=(self.nlight*3)), (self.nlight, 3, ))
        arr.setflags(write=False)
        return arr
    
    @light_dir0.setter
    def light_dir0(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.light_dir0, val_ptr, self.nlight*3 * sizeof(c_double))
    
    @property
    def light_attenuation(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_attenuation, dtype=np.float, count=(self.nlight*3)), (self.nlight, 3, ))
        arr.setflags(write=False)
        return arr
    
    @light_attenuation.setter
    def light_attenuation(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.light_attenuation, val_ptr, self.nlight*3 * sizeof(c_float))
    
    @property
    def light_cutoff(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_cutoff, dtype=np.float, count=(self.nlight*1)), (self.nlight, 1, ))
        arr.setflags(write=False)
        return arr
    
    @light_cutoff.setter
    def light_cutoff(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.light_cutoff, val_ptr, self.nlight*1 * sizeof(c_float))
    
    @property
    def light_exponent(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_exponent, dtype=np.float, count=(self.nlight*1)), (self.nlight, 1, ))
        arr.setflags(write=False)
        return arr
    
    @light_exponent.setter
    def light_exponent(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.light_exponent, val_ptr, self.nlight*1 * sizeof(c_float))
    
    @property
    def light_ambient(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_ambient, dtype=np.float, count=(self.nlight*3)), (self.nlight, 3, ))
        arr.setflags(write=False)
        return arr
    
    @light_ambient.setter
    def light_ambient(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.light_ambient, val_ptr, self.nlight*3 * sizeof(c_float))
    
    @property
    def light_diffuse(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_diffuse, dtype=np.float, count=(self.nlight*3)), (self.nlight, 3, ))
        arr.setflags(write=False)
        return arr
    
    @light_diffuse.setter
    def light_diffuse(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.light_diffuse, val_ptr, self.nlight*3 * sizeof(c_float))
    
    @property
    def light_specular(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.light_specular, dtype=np.float, count=(self.nlight*3)), (self.nlight, 3, ))
        arr.setflags(write=False)
        return arr
    
    @light_specular.setter
    def light_specular(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.light_specular, val_ptr, self.nlight*3 * sizeof(c_float))
    
    @property
    def mesh_faceadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mesh_faceadr, dtype=np.int, count=(self.nmesh*1)), (self.nmesh, 1, ))
        arr.setflags(write=False)
        return arr
    
    @mesh_faceadr.setter
    def mesh_faceadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.mesh_faceadr, val_ptr, self.nmesh*1 * sizeof(c_int))
    
    @property
    def mesh_facenum(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mesh_facenum, dtype=np.int, count=(self.nmesh*1)), (self.nmesh, 1, ))
        arr.setflags(write=False)
        return arr
    
    @mesh_facenum.setter
    def mesh_facenum(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.mesh_facenum, val_ptr, self.nmesh*1 * sizeof(c_int))
    
    @property
    def mesh_vertadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mesh_vertadr, dtype=np.int, count=(self.nmesh*1)), (self.nmesh, 1, ))
        arr.setflags(write=False)
        return arr
    
    @mesh_vertadr.setter
    def mesh_vertadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.mesh_vertadr, val_ptr, self.nmesh*1 * sizeof(c_int))
    
    @property
    def mesh_vertnum(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mesh_vertnum, dtype=np.int, count=(self.nmesh*1)), (self.nmesh, 1, ))
        arr.setflags(write=False)
        return arr
    
    @mesh_vertnum.setter
    def mesh_vertnum(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.mesh_vertnum, val_ptr, self.nmesh*1 * sizeof(c_int))
    
    @property
    def mesh_graphadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mesh_graphadr, dtype=np.int, count=(self.nmesh*1)), (self.nmesh, 1, ))
        arr.setflags(write=False)
        return arr
    
    @mesh_graphadr.setter
    def mesh_graphadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.mesh_graphadr, val_ptr, self.nmesh*1 * sizeof(c_int))
    
    @property
    def mesh_vert(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mesh_vert, dtype=np.float, count=(self.nmeshvert*3)), (self.nmeshvert, 3, ))
        arr.setflags(write=False)
        return arr
    
    @mesh_vert.setter
    def mesh_vert(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.mesh_vert, val_ptr, self.nmeshvert*3 * sizeof(c_float))
    
    @property
    def mesh_normal(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mesh_normal, dtype=np.float, count=(self.nmeshvert*3)), (self.nmeshvert, 3, ))
        arr.setflags(write=False)
        return arr
    
    @mesh_normal.setter
    def mesh_normal(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.mesh_normal, val_ptr, self.nmeshvert*3 * sizeof(c_float))
    
    @property
    def mesh_face(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mesh_face, dtype=np.int, count=(self.nmeshface*3)), (self.nmeshface, 3, ))
        arr.setflags(write=False)
        return arr
    
    @mesh_face.setter
    def mesh_face(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.mesh_face, val_ptr, self.nmeshface*3 * sizeof(c_int))
    
    @property
    def mesh_graph(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mesh_graph, dtype=np.int, count=(self.nmeshgraph*1)), (self.nmeshgraph, 1, ))
        arr.setflags(write=False)
        return arr
    
    @mesh_graph.setter
    def mesh_graph(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.mesh_graph, val_ptr, self.nmeshgraph*1 * sizeof(c_int))
    
    @property
    def hfield_size(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.hfield_size, dtype=np.double, count=(self.nhfield*4)), (self.nhfield, 4, ))
        arr.setflags(write=False)
        return arr
    
    @hfield_size.setter
    def hfield_size(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.hfield_size, val_ptr, self.nhfield*4 * sizeof(c_double))
    
    @property
    def hfield_nrow(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.hfield_nrow, dtype=np.int, count=(self.nhfield*1)), (self.nhfield, 1, ))
        arr.setflags(write=False)
        return arr
    
    @hfield_nrow.setter
    def hfield_nrow(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.hfield_nrow, val_ptr, self.nhfield*1 * sizeof(c_int))
    
    @property
    def hfield_ncol(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.hfield_ncol, dtype=np.int, count=(self.nhfield*1)), (self.nhfield, 1, ))
        arr.setflags(write=False)
        return arr
    
    @hfield_ncol.setter
    def hfield_ncol(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.hfield_ncol, val_ptr, self.nhfield*1 * sizeof(c_int))
    
    @property
    def hfield_adr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.hfield_adr, dtype=np.int, count=(self.nhfield*1)), (self.nhfield, 1, ))
        arr.setflags(write=False)
        return arr
    
    @hfield_adr.setter
    def hfield_adr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.hfield_adr, val_ptr, self.nhfield*1 * sizeof(c_int))
    
    @property
    def hfield_data(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.hfield_data, dtype=np.float, count=(self.nhfielddata*1)), (self.nhfielddata, 1, ))
        arr.setflags(write=False)
        return arr
    
    @hfield_data.setter
    def hfield_data(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.hfield_data, val_ptr, self.nhfielddata*1 * sizeof(c_float))
    
    @property
    def tex_type(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tex_type, dtype=np.int, count=(self.ntex*1)), (self.ntex, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tex_type.setter
    def tex_type(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.tex_type, val_ptr, self.ntex*1 * sizeof(c_int))
    
    @property
    def tex_height(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tex_height, dtype=np.int, count=(self.ntex*1)), (self.ntex, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tex_height.setter
    def tex_height(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.tex_height, val_ptr, self.ntex*1 * sizeof(c_int))
    
    @property
    def tex_width(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tex_width, dtype=np.int, count=(self.ntex*1)), (self.ntex, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tex_width.setter
    def tex_width(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.tex_width, val_ptr, self.ntex*1 * sizeof(c_int))
    
    @property
    def tex_adr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tex_adr, dtype=np.int, count=(self.ntex*1)), (self.ntex, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tex_adr.setter
    def tex_adr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.tex_adr, val_ptr, self.ntex*1 * sizeof(c_int))
    
    @property
    def tex_rgb(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tex_rgb, dtype=np.uint8, count=(self.ntexdata*1)), (self.ntexdata, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tex_rgb.setter
    def tex_rgb(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.tex_rgb, val_ptr, self.ntexdata*1 * sizeof(c_ubyte))
    
    @property
    def mat_texid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mat_texid, dtype=np.int, count=(self.nmat*1)), (self.nmat, 1, ))
        arr.setflags(write=False)
        return arr
    
    @mat_texid.setter
    def mat_texid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.mat_texid, val_ptr, self.nmat*1 * sizeof(c_int))
    
    @property
    def mat_texuniform(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mat_texuniform, dtype=np.uint8, count=(self.nmat*1)), (self.nmat, 1, ))
        arr.setflags(write=False)
        return arr
    
    @mat_texuniform.setter
    def mat_texuniform(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.mat_texuniform, val_ptr, self.nmat*1 * sizeof(c_ubyte))
    
    @property
    def mat_texrepeat(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mat_texrepeat, dtype=np.float, count=(self.nmat*2)), (self.nmat, 2, ))
        arr.setflags(write=False)
        return arr
    
    @mat_texrepeat.setter
    def mat_texrepeat(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.mat_texrepeat, val_ptr, self.nmat*2 * sizeof(c_float))
    
    @property
    def mat_emission(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mat_emission, dtype=np.float, count=(self.nmat*1)), (self.nmat, 1, ))
        arr.setflags(write=False)
        return arr
    
    @mat_emission.setter
    def mat_emission(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.mat_emission, val_ptr, self.nmat*1 * sizeof(c_float))
    
    @property
    def mat_specular(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mat_specular, dtype=np.float, count=(self.nmat*1)), (self.nmat, 1, ))
        arr.setflags(write=False)
        return arr
    
    @mat_specular.setter
    def mat_specular(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.mat_specular, val_ptr, self.nmat*1 * sizeof(c_float))
    
    @property
    def mat_shininess(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mat_shininess, dtype=np.float, count=(self.nmat*1)), (self.nmat, 1, ))
        arr.setflags(write=False)
        return arr
    
    @mat_shininess.setter
    def mat_shininess(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.mat_shininess, val_ptr, self.nmat*1 * sizeof(c_float))
    
    @property
    def mat_reflectance(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mat_reflectance, dtype=np.float, count=(self.nmat*1)), (self.nmat, 1, ))
        arr.setflags(write=False)
        return arr
    
    @mat_reflectance.setter
    def mat_reflectance(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.mat_reflectance, val_ptr, self.nmat*1 * sizeof(c_float))
    
    @property
    def mat_rgba(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.mat_rgba, dtype=np.float, count=(self.nmat*4)), (self.nmat, 4, ))
        arr.setflags(write=False)
        return arr
    
    @mat_rgba.setter
    def mat_rgba(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.mat_rgba, val_ptr, self.nmat*4 * sizeof(c_float))
    
    @property
    def pair_dim(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.pair_dim, dtype=np.int, count=(self.npair*1)), (self.npair, 1, ))
        arr.setflags(write=False)
        return arr
    
    @pair_dim.setter
    def pair_dim(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.pair_dim, val_ptr, self.npair*1 * sizeof(c_int))
    
    @property
    def pair_geom1(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.pair_geom1, dtype=np.int, count=(self.npair*1)), (self.npair, 1, ))
        arr.setflags(write=False)
        return arr
    
    @pair_geom1.setter
    def pair_geom1(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.pair_geom1, val_ptr, self.npair*1 * sizeof(c_int))
    
    @property
    def pair_geom2(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.pair_geom2, dtype=np.int, count=(self.npair*1)), (self.npair, 1, ))
        arr.setflags(write=False)
        return arr
    
    @pair_geom2.setter
    def pair_geom2(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.pair_geom2, val_ptr, self.npair*1 * sizeof(c_int))
    
    @property
    def pair_signature(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.pair_signature, dtype=np.int, count=(self.npair*1)), (self.npair, 1, ))
        arr.setflags(write=False)
        return arr
    
    @pair_signature.setter
    def pair_signature(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.pair_signature, val_ptr, self.npair*1 * sizeof(c_int))
    
    @property
    def pair_solref(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.pair_solref, dtype=np.double, count=(self.npair*2)), (self.npair, 2, ))
        arr.setflags(write=False)
        return arr
    
    @pair_solref.setter
    def pair_solref(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.pair_solref, val_ptr, self.npair*2 * sizeof(c_double))
    
    @property
    def pair_solimp(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.pair_solimp, dtype=np.double, count=(self.npair*3)), (self.npair, 3, ))
        arr.setflags(write=False)
        return arr
    
    @pair_solimp.setter
    def pair_solimp(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.pair_solimp, val_ptr, self.npair*3 * sizeof(c_double))
    
    @property
    def pair_margin(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.pair_margin, dtype=np.double, count=(self.npair*1)), (self.npair, 1, ))
        arr.setflags(write=False)
        return arr
    
    @pair_margin.setter
    def pair_margin(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.pair_margin, val_ptr, self.npair*1 * sizeof(c_double))
    
    @property
    def pair_gap(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.pair_gap, dtype=np.double, count=(self.npair*1)), (self.npair, 1, ))
        arr.setflags(write=False)
        return arr
    
    @pair_gap.setter
    def pair_gap(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.pair_gap, val_ptr, self.npair*1 * sizeof(c_double))
    
    @property
    def pair_friction(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.pair_friction, dtype=np.double, count=(self.npair*5)), (self.npair, 5, ))
        arr.setflags(write=False)
        return arr
    
    @pair_friction.setter
    def pair_friction(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.pair_friction, val_ptr, self.npair*5 * sizeof(c_double))
    
    @property
    def exclude_signature(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.exclude_signature, dtype=np.int, count=(self.nexclude*1)), (self.nexclude, 1, ))
        arr.setflags(write=False)
        return arr
    
    @exclude_signature.setter
    def exclude_signature(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.exclude_signature, val_ptr, self.nexclude*1 * sizeof(c_int))
    
    @property
    def eq_type(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.eq_type, dtype=np.int, count=(self.neq*1)), (self.neq, 1, ))
        arr.setflags(write=False)
        return arr
    
    @eq_type.setter
    def eq_type(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.eq_type, val_ptr, self.neq*1 * sizeof(c_int))
    
    @property
    def eq_obj1id(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.eq_obj1id, dtype=np.int, count=(self.neq*1)), (self.neq, 1, ))
        arr.setflags(write=False)
        return arr
    
    @eq_obj1id.setter
    def eq_obj1id(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.eq_obj1id, val_ptr, self.neq*1 * sizeof(c_int))
    
    @property
    def eq_obj2id(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.eq_obj2id, dtype=np.int, count=(self.neq*1)), (self.neq, 1, ))
        arr.setflags(write=False)
        return arr
    
    @eq_obj2id.setter
    def eq_obj2id(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.eq_obj2id, val_ptr, self.neq*1 * sizeof(c_int))
    
    @property
    def eq_active(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.eq_active, dtype=np.uint8, count=(self.neq*1)), (self.neq, 1, ))
        arr.setflags(write=False)
        return arr
    
    @eq_active.setter
    def eq_active(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.eq_active, val_ptr, self.neq*1 * sizeof(c_ubyte))
    
    @property
    def eq_solref(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.eq_solref, dtype=np.double, count=(self.neq*2)), (self.neq, 2, ))
        arr.setflags(write=False)
        return arr
    
    @eq_solref.setter
    def eq_solref(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.eq_solref, val_ptr, self.neq*2 * sizeof(c_double))
    
    @property
    def eq_solimp(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.eq_solimp, dtype=np.double, count=(self.neq*3)), (self.neq, 3, ))
        arr.setflags(write=False)
        return arr
    
    @eq_solimp.setter
    def eq_solimp(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.eq_solimp, val_ptr, self.neq*3 * sizeof(c_double))
    
    @property
    def eq_data(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.eq_data, dtype=np.double, count=(self.neq*7)), (self.neq, 7, ))
        arr.setflags(write=False)
        return arr
    
    @eq_data.setter
    def eq_data(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.eq_data, val_ptr, self.neq*7 * sizeof(c_double))
    
    @property
    def tendon_adr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_adr, dtype=np.int, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_adr.setter
    def tendon_adr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.tendon_adr, val_ptr, self.ntendon*1 * sizeof(c_int))
    
    @property
    def tendon_num(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_num, dtype=np.int, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_num.setter
    def tendon_num(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.tendon_num, val_ptr, self.ntendon*1 * sizeof(c_int))
    
    @property
    def tendon_matid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_matid, dtype=np.int, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_matid.setter
    def tendon_matid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.tendon_matid, val_ptr, self.ntendon*1 * sizeof(c_int))
    
    @property
    def tendon_limited(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_limited, dtype=np.uint8, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_limited.setter
    def tendon_limited(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.tendon_limited, val_ptr, self.ntendon*1 * sizeof(c_ubyte))
    
    @property
    def tendon_frictional(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_frictional, dtype=np.uint8, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_frictional.setter
    def tendon_frictional(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.tendon_frictional, val_ptr, self.ntendon*1 * sizeof(c_ubyte))
    
    @property
    def tendon_width(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_width, dtype=np.double, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_width.setter
    def tendon_width(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_width, val_ptr, self.ntendon*1 * sizeof(c_double))
    
    @property
    def tendon_solref_lim(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_solref_lim, dtype=np.double, count=(self.ntendon*2)), (self.ntendon, 2, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_solref_lim.setter
    def tendon_solref_lim(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_solref_lim, val_ptr, self.ntendon*2 * sizeof(c_double))
    
    @property
    def tendon_solimp_lim(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_solimp_lim, dtype=np.double, count=(self.ntendon*3)), (self.ntendon, 3, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_solimp_lim.setter
    def tendon_solimp_lim(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_solimp_lim, val_ptr, self.ntendon*3 * sizeof(c_double))
    
    @property
    def tendon_solref_fri(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_solref_fri, dtype=np.double, count=(self.ntendon*2)), (self.ntendon, 2, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_solref_fri.setter
    def tendon_solref_fri(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_solref_fri, val_ptr, self.ntendon*2 * sizeof(c_double))
    
    @property
    def tendon_solimp_fri(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_solimp_fri, dtype=np.double, count=(self.ntendon*3)), (self.ntendon, 3, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_solimp_fri.setter
    def tendon_solimp_fri(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_solimp_fri, val_ptr, self.ntendon*3 * sizeof(c_double))
    
    @property
    def tendon_range(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_range, dtype=np.double, count=(self.ntendon*2)), (self.ntendon, 2, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_range.setter
    def tendon_range(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_range, val_ptr, self.ntendon*2 * sizeof(c_double))
    
    @property
    def tendon_margin(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_margin, dtype=np.double, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_margin.setter
    def tendon_margin(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_margin, val_ptr, self.ntendon*1 * sizeof(c_double))
    
    @property
    def tendon_stiffness(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_stiffness, dtype=np.double, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_stiffness.setter
    def tendon_stiffness(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_stiffness, val_ptr, self.ntendon*1 * sizeof(c_double))
    
    @property
    def tendon_damping(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_damping, dtype=np.double, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_damping.setter
    def tendon_damping(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_damping, val_ptr, self.ntendon*1 * sizeof(c_double))
    
    @property
    def tendon_frictionloss(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_frictionloss, dtype=np.double, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_frictionloss.setter
    def tendon_frictionloss(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_frictionloss, val_ptr, self.ntendon*1 * sizeof(c_double))
    
    @property
    def tendon_lengthspring(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_lengthspring, dtype=np.double, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_lengthspring.setter
    def tendon_lengthspring(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_lengthspring, val_ptr, self.ntendon*1 * sizeof(c_double))
    
    @property
    def tendon_length0(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_length0, dtype=np.double, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_length0.setter
    def tendon_length0(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_length0, val_ptr, self.ntendon*1 * sizeof(c_double))
    
    @property
    def tendon_invweight0(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_invweight0, dtype=np.double, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_invweight0.setter
    def tendon_invweight0(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_invweight0, val_ptr, self.ntendon*1 * sizeof(c_double))
    
    @property
    def tendon_user(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_user, dtype=np.double, count=(self.ntendon*self.nuser_tendon)), (self.ntendon, self.nuser_tendon, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_user.setter
    def tendon_user(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.tendon_user, val_ptr, self.ntendon*self.nuser_tendon * sizeof(c_double))
    
    @property
    def tendon_rgba(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.tendon_rgba, dtype=np.float, count=(self.ntendon*4)), (self.ntendon, 4, ))
        arr.setflags(write=False)
        return arr
    
    @tendon_rgba.setter
    def tendon_rgba(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_float))
        memmove(self._wrapped.contents.tendon_rgba, val_ptr, self.ntendon*4 * sizeof(c_float))
    
    @property
    def wrap_type(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.wrap_type, dtype=np.int, count=(self.nwrap*1)), (self.nwrap, 1, ))
        arr.setflags(write=False)
        return arr
    
    @wrap_type.setter
    def wrap_type(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.wrap_type, val_ptr, self.nwrap*1 * sizeof(c_int))
    
    @property
    def wrap_objid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.wrap_objid, dtype=np.int, count=(self.nwrap*1)), (self.nwrap, 1, ))
        arr.setflags(write=False)
        return arr
    
    @wrap_objid.setter
    def wrap_objid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.wrap_objid, val_ptr, self.nwrap*1 * sizeof(c_int))
    
    @property
    def wrap_prm(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.wrap_prm, dtype=np.double, count=(self.nwrap*1)), (self.nwrap, 1, ))
        arr.setflags(write=False)
        return arr
    
    @wrap_prm.setter
    def wrap_prm(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.wrap_prm, val_ptr, self.nwrap*1 * sizeof(c_double))
    
    @property
    def actuator_trntype(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_trntype, dtype=np.int, count=(self.nu*1)), (self.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_trntype.setter
    def actuator_trntype(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.actuator_trntype, val_ptr, self.nu*1 * sizeof(c_int))
    
    @property
    def actuator_dyntype(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_dyntype, dtype=np.int, count=(self.nu*1)), (self.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_dyntype.setter
    def actuator_dyntype(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.actuator_dyntype, val_ptr, self.nu*1 * sizeof(c_int))
    
    @property
    def actuator_gaintype(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_gaintype, dtype=np.int, count=(self.nu*1)), (self.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_gaintype.setter
    def actuator_gaintype(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.actuator_gaintype, val_ptr, self.nu*1 * sizeof(c_int))
    
    @property
    def actuator_biastype(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_biastype, dtype=np.int, count=(self.nu*1)), (self.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_biastype.setter
    def actuator_biastype(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.actuator_biastype, val_ptr, self.nu*1 * sizeof(c_int))
    
    @property
    def actuator_trnid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_trnid, dtype=np.int, count=(self.nu*2)), (self.nu, 2, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_trnid.setter
    def actuator_trnid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.actuator_trnid, val_ptr, self.nu*2 * sizeof(c_int))
    
    @property
    def actuator_ctrllimited(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_ctrllimited, dtype=np.uint8, count=(self.nu*1)), (self.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_ctrllimited.setter
    def actuator_ctrllimited(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.actuator_ctrllimited, val_ptr, self.nu*1 * sizeof(c_ubyte))
    
    @property
    def actuator_forcelimited(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_forcelimited, dtype=np.uint8, count=(self.nu*1)), (self.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_forcelimited.setter
    def actuator_forcelimited(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_ubyte))
        memmove(self._wrapped.contents.actuator_forcelimited, val_ptr, self.nu*1 * sizeof(c_ubyte))
    
    @property
    def actuator_dynprm(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_dynprm, dtype=np.double, count=(self.nu*3)), (self.nu, 3, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_dynprm.setter
    def actuator_dynprm(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_dynprm, val_ptr, self.nu*3 * sizeof(c_double))
    
    @property
    def actuator_gainprm(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_gainprm, dtype=np.double, count=(self.nu*3)), (self.nu, 3, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_gainprm.setter
    def actuator_gainprm(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_gainprm, val_ptr, self.nu*3 * sizeof(c_double))
    
    @property
    def actuator_biasprm(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_biasprm, dtype=np.double, count=(self.nu*3)), (self.nu, 3, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_biasprm.setter
    def actuator_biasprm(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_biasprm, val_ptr, self.nu*3 * sizeof(c_double))
    
    @property
    def actuator_ctrlrange(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_ctrlrange, dtype=np.double, count=(self.nu*2)), (self.nu, 2, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_ctrlrange.setter
    def actuator_ctrlrange(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_ctrlrange, val_ptr, self.nu*2 * sizeof(c_double))
    
    @property
    def actuator_forcerange(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_forcerange, dtype=np.double, count=(self.nu*2)), (self.nu, 2, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_forcerange.setter
    def actuator_forcerange(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_forcerange, val_ptr, self.nu*2 * sizeof(c_double))
    
    @property
    def actuator_gear(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_gear, dtype=np.double, count=(self.nu*6)), (self.nu, 6, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_gear.setter
    def actuator_gear(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_gear, val_ptr, self.nu*6 * sizeof(c_double))
    
    @property
    def actuator_cranklength(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_cranklength, dtype=np.double, count=(self.nu*1)), (self.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_cranklength.setter
    def actuator_cranklength(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_cranklength, val_ptr, self.nu*1 * sizeof(c_double))
    
    @property
    def actuator_invweight0(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_invweight0, dtype=np.double, count=(self.nu*1)), (self.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_invweight0.setter
    def actuator_invweight0(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_invweight0, val_ptr, self.nu*1 * sizeof(c_double))
    
    @property
    def actuator_length0(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_length0, dtype=np.double, count=(self.nu*1)), (self.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_length0.setter
    def actuator_length0(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_length0, val_ptr, self.nu*1 * sizeof(c_double))
    
    @property
    def actuator_lengthrange(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_lengthrange, dtype=np.double, count=(self.nu*2)), (self.nu, 2, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_lengthrange.setter
    def actuator_lengthrange(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_lengthrange, val_ptr, self.nu*2 * sizeof(c_double))
    
    @property
    def actuator_user(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.actuator_user, dtype=np.double, count=(self.nu*self.nuser_actuator)), (self.nu, self.nuser_actuator, ))
        arr.setflags(write=False)
        return arr
    
    @actuator_user.setter
    def actuator_user(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.actuator_user, val_ptr, self.nu*self.nuser_actuator * sizeof(c_double))
    
    @property
    def sensor_type(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.sensor_type, dtype=np.int, count=(self.nsensor*1)), (self.nsensor, 1, ))
        arr.setflags(write=False)
        return arr
    
    @sensor_type.setter
    def sensor_type(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.sensor_type, val_ptr, self.nsensor*1 * sizeof(c_int))
    
    @property
    def sensor_objid(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.sensor_objid, dtype=np.int, count=(self.nsensor*1)), (self.nsensor, 1, ))
        arr.setflags(write=False)
        return arr
    
    @sensor_objid.setter
    def sensor_objid(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.sensor_objid, val_ptr, self.nsensor*1 * sizeof(c_int))
    
    @property
    def sensor_dim(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.sensor_dim, dtype=np.int, count=(self.nsensor*1)), (self.nsensor, 1, ))
        arr.setflags(write=False)
        return arr
    
    @sensor_dim.setter
    def sensor_dim(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.sensor_dim, val_ptr, self.nsensor*1 * sizeof(c_int))
    
    @property
    def sensor_adr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.sensor_adr, dtype=np.int, count=(self.nsensor*1)), (self.nsensor, 1, ))
        arr.setflags(write=False)
        return arr
    
    @sensor_adr.setter
    def sensor_adr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.sensor_adr, val_ptr, self.nsensor*1 * sizeof(c_int))
    
    @property
    def sensor_scale(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.sensor_scale, dtype=np.double, count=(self.nsensor*1)), (self.nsensor, 1, ))
        arr.setflags(write=False)
        return arr
    
    @sensor_scale.setter
    def sensor_scale(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.sensor_scale, val_ptr, self.nsensor*1 * sizeof(c_double))
    
    @property
    def sensor_user(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.sensor_user, dtype=np.double, count=(self.nsensor*self.nuser_sensor)), (self.nsensor, self.nuser_sensor, ))
        arr.setflags(write=False)
        return arr
    
    @sensor_user.setter
    def sensor_user(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.sensor_user, val_ptr, self.nsensor*self.nuser_sensor * sizeof(c_double))
    
    @property
    def numeric_adr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.numeric_adr, dtype=np.int, count=(self.nnumeric*1)), (self.nnumeric, 1, ))
        arr.setflags(write=False)
        return arr
    
    @numeric_adr.setter
    def numeric_adr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.numeric_adr, val_ptr, self.nnumeric*1 * sizeof(c_int))
    
    @property
    def numeric_size(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.numeric_size, dtype=np.int, count=(self.nnumeric*1)), (self.nnumeric, 1, ))
        arr.setflags(write=False)
        return arr
    
    @numeric_size.setter
    def numeric_size(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.numeric_size, val_ptr, self.nnumeric*1 * sizeof(c_int))
    
    @property
    def numeric_data(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.numeric_data, dtype=np.double, count=(self.nnumericdata*1)), (self.nnumericdata, 1, ))
        arr.setflags(write=False)
        return arr
    
    @numeric_data.setter
    def numeric_data(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.numeric_data, val_ptr, self.nnumericdata*1 * sizeof(c_double))
    
    @property
    def text_adr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.text_adr, dtype=np.int, count=(self.ntext*1)), (self.ntext, 1, ))
        arr.setflags(write=False)
        return arr
    
    @text_adr.setter
    def text_adr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.text_adr, val_ptr, self.ntext*1 * sizeof(c_int))
    
    @property
    def text_data(self):
        return self._wrapped.contents.text_data
    
    @property
    def key_time(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.key_time, dtype=np.double, count=(self.nkey*1)), (self.nkey, 1, ))
        arr.setflags(write=False)
        return arr
    
    @key_time.setter
    def key_time(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.key_time, val_ptr, self.nkey*1 * sizeof(c_double))
    
    @property
    def key_qpos(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.key_qpos, dtype=np.double, count=(self.nkey*self.nq)), (self.nkey, self.nq, ))
        arr.setflags(write=False)
        return arr
    
    @key_qpos.setter
    def key_qpos(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.key_qpos, val_ptr, self.nkey*self.nq * sizeof(c_double))
    
    @property
    def key_qvel(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.key_qvel, dtype=np.double, count=(self.nkey*self.nv)), (self.nkey, self.nv, ))
        arr.setflags(write=False)
        return arr
    
    @key_qvel.setter
    def key_qvel(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.key_qvel, val_ptr, self.nkey*self.nv * sizeof(c_double))
    
    @property
    def key_act(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.key_act, dtype=np.double, count=(self.nkey*self.na)), (self.nkey, self.na, ))
        arr.setflags(write=False)
        return arr
    
    @key_act.setter
    def key_act(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_double))
        memmove(self._wrapped.contents.key_act, val_ptr, self.nkey*self.na * sizeof(c_double))
    
    @property
    def name_bodyadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_bodyadr, dtype=np.int, count=(self.nbody*1)), (self.nbody, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_bodyadr.setter
    def name_bodyadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_bodyadr, val_ptr, self.nbody*1 * sizeof(c_int))
    
    @property
    def name_jntadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_jntadr, dtype=np.int, count=(self.njnt*1)), (self.njnt, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_jntadr.setter
    def name_jntadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_jntadr, val_ptr, self.njnt*1 * sizeof(c_int))
    
    @property
    def name_geomadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_geomadr, dtype=np.int, count=(self.ngeom*1)), (self.ngeom, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_geomadr.setter
    def name_geomadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_geomadr, val_ptr, self.ngeom*1 * sizeof(c_int))
    
    @property
    def name_siteadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_siteadr, dtype=np.int, count=(self.nsite*1)), (self.nsite, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_siteadr.setter
    def name_siteadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_siteadr, val_ptr, self.nsite*1 * sizeof(c_int))
    
    @property
    def name_camadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_camadr, dtype=np.int, count=(self.ncam*1)), (self.ncam, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_camadr.setter
    def name_camadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_camadr, val_ptr, self.ncam*1 * sizeof(c_int))
    
    @property
    def name_lightadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_lightadr, dtype=np.int, count=(self.nlight*1)), (self.nlight, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_lightadr.setter
    def name_lightadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_lightadr, val_ptr, self.nlight*1 * sizeof(c_int))
    
    @property
    def name_meshadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_meshadr, dtype=np.int, count=(self.nmesh*1)), (self.nmesh, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_meshadr.setter
    def name_meshadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_meshadr, val_ptr, self.nmesh*1 * sizeof(c_int))
    
    @property
    def name_hfieldadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_hfieldadr, dtype=np.int, count=(self.nhfield*1)), (self.nhfield, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_hfieldadr.setter
    def name_hfieldadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_hfieldadr, val_ptr, self.nhfield*1 * sizeof(c_int))
    
    @property
    def name_texadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_texadr, dtype=np.int, count=(self.ntex*1)), (self.ntex, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_texadr.setter
    def name_texadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_texadr, val_ptr, self.ntex*1 * sizeof(c_int))
    
    @property
    def name_matadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_matadr, dtype=np.int, count=(self.nmat*1)), (self.nmat, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_matadr.setter
    def name_matadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_matadr, val_ptr, self.nmat*1 * sizeof(c_int))
    
    @property
    def name_eqadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_eqadr, dtype=np.int, count=(self.neq*1)), (self.neq, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_eqadr.setter
    def name_eqadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_eqadr, val_ptr, self.neq*1 * sizeof(c_int))
    
    @property
    def name_tendonadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_tendonadr, dtype=np.int, count=(self.ntendon*1)), (self.ntendon, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_tendonadr.setter
    def name_tendonadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_tendonadr, val_ptr, self.ntendon*1 * sizeof(c_int))
    
    @property
    def name_actuatoradr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_actuatoradr, dtype=np.int, count=(self.nu*1)), (self.nu, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_actuatoradr.setter
    def name_actuatoradr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_actuatoradr, val_ptr, self.nu*1 * sizeof(c_int))
    
    @property
    def name_sensoradr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_sensoradr, dtype=np.int, count=(self.nsensor*1)), (self.nsensor, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_sensoradr.setter
    def name_sensoradr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_sensoradr, val_ptr, self.nsensor*1 * sizeof(c_int))
    
    @property
    def name_numericadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_numericadr, dtype=np.int, count=(self.nnumeric*1)), (self.nnumeric, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_numericadr.setter
    def name_numericadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_numericadr, val_ptr, self.nnumeric*1 * sizeof(c_int))
    
    @property
    def name_textadr(self):
        arr = np.reshape(np.fromiter(self._wrapped.contents.name_textadr, dtype=np.int, count=(self.ntext*1)), (self.ntext, 1, ))
        arr.setflags(write=False)
        return arr
    
    @name_textadr.setter
    def name_textadr(self, value):
        val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(c_int))
        memmove(self._wrapped.contents.name_textadr, val_ptr, self.ntext*1 * sizeof(c_int))
    
    @property
    def names(self):
        return self._wrapped.contents.names
