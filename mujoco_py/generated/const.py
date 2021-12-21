# Automatically generated. Do not modify!

###### const from defines ######
MINVAL = 1e-15                      # minimum value in any denominator
PI = 3.141592653589793              # 
MAXVAL = 10000000000.0              # maximum value in qpos, qvel, qacc
MINMU = 1e-05                       # minimum friction coefficient
MINIMP = 0.0001                     # minimum constraint impedance
MAXIMP = 0.9999                     # maximum constraint impedance
MAXCONPAIR = 50.0                   # maximum number of contacts per geom pair
MAXVFS = 2000.0                     # maximum number of files in virtual file system
MAXVFSNAME = 1000.0                 # maximum filename size in virtual file system
NEQDATA = 7.0                       # number of eq_data fields
NDYN = 10.0                         # number of actuator dynamics parameters
NGAIN = 10.0                        # number of actuator gain parameters
NBIAS = 10.0                        # number of actuator bias parameters
NREF = 2.0                          # number of solver reference parameters
NIMP = 5.0                          # number of solver impedance parameters
NSOLVER = 1000.0                    # size of mjData.solver_XXX arrays
NGROUP = 6.0                        # number of geom, site, joint groups with visflags
MAXOVERLAY = 500.0                  # maximum number of characters in overlay text
MAXLINE = 100.0                     # maximum number of lines per plot
MAXLINEPNT = 1000.0                 # maximum number points per line
MAXPLANEGRID = 200.0                # maximum number of grid divisions for plane
NAUX = 10.0                         # number of auxiliary buffers
MAXTEXTURE = 1000.0                 # maximum number of textures
MAXUISECT = 10.0                    # maximum number of sections
MAXUIITEM = 80.0                    # maximum number of items per section
MAXUITEXT = 300.0                   # maximum number of chars in edittext and other
MAXUINAME = 40.0                    # maximum number of chars in name
MAXUIMULTI = 35.0                   # maximum number of radio/select items in group
MAXUIEDIT = 7.0                     # maximum number of elements in edit list
MAXUIRECT = 25.0                    # maximum number of rectangles
SEPCLOSED = 1000.0                  # closed state of adjustable separator
KEY_ESCAPE = 256.0                  # 
KEY_ENTER = 257.0                   # 
KEY_TAB = 258.0                     # 
KEY_BACKSPACE = 259.0               # 
KEY_INSERT = 260.0                  # 
KEY_DELETE = 261.0                  # 
KEY_RIGHT = 262.0                   # 
KEY_LEFT = 263.0                    # 
KEY_DOWN = 264.0                    # 
KEY_UP = 265.0                      # 
KEY_PAGE_UP = 266.0                 # 
KEY_PAGE_DOWN = 267.0               # 
KEY_HOME = 268.0                    # 
KEY_END = 269.0                     # 
KEY_F1 = 290.0                      # 
KEY_F2 = 291.0                      # 
KEY_F3 = 292.0                      # 
KEY_F4 = 293.0                      # 
KEY_F5 = 294.0                      # 
KEY_F6 = 295.0                      # 
KEY_F7 = 296.0                      # 
KEY_F8 = 297.0                      # 
KEY_F9 = 298.0                      # 
KEY_F10 = 299.0                     # 
KEY_F11 = 300.0                     # 
KEY_F12 = 301.0                     # 

###### const from enums ######

 # _mjtDisableBit
DSBL_CONSTRAINT = 1
DSBL_EQUALITY = 2
DSBL_FRICTIONLOSS = 4
DSBL_LIMIT = 8
DSBL_CONTACT = 16
DSBL_PASSIVE = 32
DSBL_GRAVITY = 64
DSBL_CLAMPCTRL = 128
DSBL_WARMSTART = 256
DSBL_FILTERPARENT = 512
DSBL_ACTUATION = 1024
DSBL_REFSAFE = 2048
NDISABLE = 12

 # _mjtEnableBit
ENBL_OVERRIDE = 1
ENBL_ENERGY = 2
ENBL_FWDINV = 4
ENBL_SENSORNOISE = 8
NENABLE = 4

 # _mjtJoint
JNT_FREE = 0
JNT_BALL = 1
JNT_SLIDE = 2
JNT_HINGE = 3

 # _mjtGeom
GEOM_PLANE = 0
GEOM_HFIELD = 1
GEOM_SPHERE = 2
GEOM_CAPSULE = 3
GEOM_ELLIPSOID = 4
GEOM_CYLINDER = 5
GEOM_BOX = 6
GEOM_MESH = 7
NGEOMTYPES = 8
GEOM_ARROW = 100
GEOM_ARROW1 = 101
GEOM_ARROW2 = 102
GEOM_LINE = 103
GEOM_SKIN = 104
GEOM_LABEL = 105
GEOM_NONE = 1001

 # _mjtCamLight
CAMLIGHT_FIXED = 0
CAMLIGHT_TRACK = 1
CAMLIGHT_TRACKCOM = 2
CAMLIGHT_TARGETBODY = 3
CAMLIGHT_TARGETBODYCOM = 4

 # _mjtTexture
TEXTURE_2D = 0
TEXTURE_CUBE = 1
TEXTURE_SKYBOX = 2

 # _mjtIntegrator
INT_EULER = 0
INT_RK4 = 1

 # _mjtCollision
COL_ALL = 0
COL_PAIR = 1
COL_DYNAMIC = 2

 # _mjtCone
CONE_PYRAMIDAL = 0
CONE_ELLIPTIC = 1

 # _mjtJacobian
JAC_DENSE = 0
JAC_SPARSE = 1
JAC_AUTO = 2

 # _mjtSolver
SOL_PGS = 0
SOL_CG = 1
SOL_NEWTON = 2

 # _mjtEq
EQ_CONNECT = 0
EQ_WELD = 1
EQ_JOINT = 2
EQ_TENDON = 3
EQ_DISTANCE = 4

 # _mjtWrap
WRAP_NONE = 0
WRAP_JOINT = 1
WRAP_PULLEY = 2
WRAP_SITE = 3
WRAP_SPHERE = 4
WRAP_CYLINDER = 5

 # _mjtTrn
TRN_JOINT = 0
TRN_JOINTINPARENT = 1
TRN_SLIDERCRANK = 2
TRN_TENDON = 3
TRN_SITE = 4
TRN_UNDEFINED = 1000

 # _mjtDyn
DYN_NONE = 0
DYN_INTEGRATOR = 1
DYN_FILTER = 2
DYN_MUSCLE = 3
DYN_USER = 4

 # _mjtGain
GAIN_FIXED = 0
GAIN_MUSCLE = 1
GAIN_USER = 2

 # _mjtBias
BIAS_NONE = 0
BIAS_AFFINE = 1
BIAS_MUSCLE = 2
BIAS_USER = 3

 # _mjtObj
OBJ_UNKNOWN = 0
OBJ_BODY = 1
OBJ_XBODY = 2
OBJ_JOINT = 3
OBJ_DOF = 4
OBJ_GEOM = 5
OBJ_SITE = 6
OBJ_CAMERA = 7
OBJ_LIGHT = 8
OBJ_MESH = 9
OBJ_SKIN = 10
OBJ_HFIELD = 11
OBJ_TEXTURE = 12
OBJ_MATERIAL = 13
OBJ_PAIR = 14
OBJ_EXCLUDE = 15
OBJ_EQUALITY = 16
OBJ_TENDON = 17
OBJ_ACTUATOR = 18
OBJ_SENSOR = 19
OBJ_NUMERIC = 20
OBJ_TEXT = 21
OBJ_TUPLE = 22
OBJ_KEY = 23

 # _mjtConstraint
CNSTR_EQUALITY = 0
CNSTR_FRICTION_DOF = 1
CNSTR_FRICTION_TENDON = 2
CNSTR_LIMIT_JOINT = 3
CNSTR_LIMIT_TENDON = 4
CNSTR_CONTACT_FRICTIONLESS = 5
CNSTR_CONTACT_PYRAMIDAL = 6
CNSTR_CONTACT_ELLIPTIC = 7

 # _mjtConstraintState
CNSTRSTATE_SATISFIED = 0
CNSTRSTATE_QUADRATIC = 1
CNSTRSTATE_LINEARNEG = 2
CNSTRSTATE_LINEARPOS = 3
CNSTRSTATE_CONE = 4

 # _mjtSensor
SENS_TOUCH = 0
SENS_ACCELEROMETER = 1
SENS_VELOCIMETER = 2
SENS_GYRO = 3
SENS_FORCE = 4
SENS_TORQUE = 5
SENS_MAGNETOMETER = 6
SENS_RANGEFINDER = 7
SENS_JOINTPOS = 8
SENS_JOINTVEL = 9
SENS_TENDONPOS = 10
SENS_TENDONVEL = 11
SENS_ACTUATORPOS = 12
SENS_ACTUATORVEL = 13
SENS_ACTUATORFRC = 14
SENS_BALLQUAT = 15
SENS_BALLANGVEL = 16
SENS_JOINTLIMITPOS = 17
SENS_JOINTLIMITVEL = 18
SENS_JOINTLIMITFRC = 19
SENS_TENDONLIMITPOS = 20
SENS_TENDONLIMITVEL = 21
SENS_TENDONLIMITFRC = 22
SENS_FRAMEPOS = 23
SENS_FRAMEQUAT = 24
SENS_FRAMEXAXIS = 25
SENS_FRAMEYAXIS = 26
SENS_FRAMEZAXIS = 27
SENS_FRAMELINVEL = 28
SENS_FRAMEANGVEL = 29
SENS_FRAMELINACC = 30
SENS_FRAMEANGACC = 31
SENS_SUBTREECOM = 32
SENS_SUBTREELINVEL = 33
SENS_SUBTREEANGMOM = 34
SENS_USER = 35

 # _mjtStage
STAGE_NONE = 0
STAGE_POS = 1
STAGE_VEL = 2
STAGE_ACC = 3

 # _mjtDataType
DATATYPE_REAL = 0
DATATYPE_POSITIVE = 1
DATATYPE_AXIS = 2
DATATYPE_QUATERNION = 3

 # _mjtLRMode
LRMODE_NONE = 0
LRMODE_MUSCLE = 1
LRMODE_MUSCLEUSER = 2
LRMODE_ALL = 3

 # _mjtWarning
WARN_INERTIA = 0
WARN_CONTACTFULL = 1
WARN_CNSTRFULL = 2
WARN_VGEOMFULL = 3
WARN_BADQPOS = 4
WARN_BADQVEL = 5
WARN_BADQACC = 6
WARN_BADCTRL = 7
NWARNING = 8

 # _mjtTimer
TIMER_STEP = 0
TIMER_FORWARD = 1
TIMER_INVERSE = 2
TIMER_POSITION = 3
TIMER_VELOCITY = 4
TIMER_ACTUATION = 5
TIMER_ACCELERATION = 6
TIMER_CONSTRAINT = 7
TIMER_POS_KINEMATICS = 8
TIMER_POS_INERTIA = 9
TIMER_POS_COLLISION = 10
TIMER_POS_MAKE = 11
TIMER_POS_PROJECT = 12
NTIMER = 13

 # _mjtCatBit
CAT_STATIC = 1
CAT_DYNAMIC = 2
CAT_DECOR = 4
CAT_ALL = 7

 # _mjtMouse
MOUSE_NONE = 0
MOUSE_ROTATE_V = 1
MOUSE_ROTATE_H = 2
MOUSE_MOVE_V = 3
MOUSE_MOVE_H = 4
MOUSE_ZOOM = 5
MOUSE_SELECT = 6

 # _mjtPertBit
PERT_TRANSLATE = 1
PERT_ROTATE = 2

 # _mjtCamera
CAMERA_FREE = 0
CAMERA_TRACKING = 1
CAMERA_FIXED = 2
CAMERA_USER = 3

 # _mjtLabel
LABEL_NONE = 0
LABEL_BODY = 1
LABEL_JOINT = 2
LABEL_GEOM = 3
LABEL_SITE = 4
LABEL_CAMERA = 5
LABEL_LIGHT = 6
LABEL_TENDON = 7
LABEL_ACTUATOR = 8
LABEL_CONSTRAINT = 9
LABEL_SKIN = 10
LABEL_SELECTION = 11
LABEL_SELPNT = 12
LABEL_CONTACTFORCE = 13
NLABEL = 14

 # _mjtFrame
FRAME_NONE = 0
FRAME_BODY = 1
FRAME_GEOM = 2
FRAME_SITE = 3
FRAME_CAMERA = 4
FRAME_LIGHT = 5
FRAME_WORLD = 6
NFRAME = 7

 # _mjtVisFlag
VIS_CONVEXHULL = 0
VIS_TEXTURE = 1
VIS_JOINT = 2
VIS_ACTUATOR = 3
VIS_CAMERA = 4
VIS_LIGHT = 5
VIS_TENDON = 6
VIS_RANGEFINDER = 7
VIS_CONSTRAINT = 8
VIS_INERTIA = 9
VIS_SCLINERTIA = 10
VIS_PERTFORCE = 11
VIS_PERTOBJ = 12
VIS_CONTACTPOINT = 13
VIS_CONTACTFORCE = 14
VIS_CONTACTSPLIT = 15
VIS_TRANSPARENT = 16
VIS_AUTOCONNECT = 17
VIS_COM = 18
VIS_SELECT = 19
VIS_STATIC = 20
VIS_SKIN = 21
NVISFLAG = 22

 # _mjtRndFlag
RND_SHADOW = 0
RND_WIREFRAME = 1
RND_REFLECTION = 2
RND_ADDITIVE = 3
RND_SKYBOX = 4
RND_FOG = 5
RND_HAZE = 6
RND_SEGMENT = 7
RND_IDCOLOR = 8
NRNDFLAG = 9

 # _mjtStereo
STEREO_NONE = 0
STEREO_QUADBUFFERED = 1
STEREO_SIDEBYSIDE = 2

 # _mjtGridPos
GRID_TOPLEFT = 0
GRID_TOPRIGHT = 1
GRID_BOTTOMLEFT = 2
GRID_BOTTOMRIGHT = 3

 # _mjtFramebuffer
FB_WINDOW = 0
FB_OFFSCREEN = 1

 # _mjtFontScale
FONTSCALE_50 = 50
FONTSCALE_100 = 100
FONTSCALE_150 = 150
FONTSCALE_200 = 200
FONTSCALE_250 = 250
FONTSCALE_300 = 300

 # _mjtFont
FONT_NORMAL = 0
FONT_SHADOW = 1
FONT_BIG = 2

 # _mjtButton
BUTTON_NONE = 0
BUTTON_LEFT = 1
BUTTON_RIGHT = 2
BUTTON_MIDDLE = 3

 # _mjtEvent
EVENT_NONE = 0
EVENT_MOVE = 1
EVENT_PRESS = 2
EVENT_RELEASE = 3
EVENT_SCROLL = 4
EVENT_KEY = 5
EVENT_RESIZE = 6

 # _mjtItem
ITEM_END = -2
ITEM_SECTION = -1
ITEM_SEPARATOR = 0
ITEM_STATIC = 1
ITEM_BUTTON = 2
ITEM_CHECKINT = 3
ITEM_CHECKBYTE = 4
ITEM_RADIO = 5
ITEM_RADIOLINE = 6
ITEM_SELECT = 7
ITEM_SLIDERINT = 8
ITEM_SLIDERNUM = 9
ITEM_EDITINT = 10
ITEM_EDITNUM = 11
ITEM_EDITTXT = 12
NITEM = 13
