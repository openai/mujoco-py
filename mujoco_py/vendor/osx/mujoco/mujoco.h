//-----------------------------------//
//  This file is part of MuJoCo.     //
//  Copyright (C) 2016 Roboti LLC.   //
//-----------------------------------//


#pragma once


// cross-platform import
#if defined(MJ_STATIC) 
    #define MJAPI
#else
    #if defined(_WIN32)
        #define MJAPI __declspec(dllimport)
    #else
        #define MJAPI
    #endif
#endif


// this is a C-API
#if defined(__cplusplus)
extern "C"
{
#endif

// header version; should match the library version as returned by mj_version()
#define mjVERSION_HEADER 131


// needed to define size_t, fabs and log10
#include "stdlib.h"
#include "math.h"


// type definitions
#include "mjmodel.h"
#include "mjdata.h"
#include "mjvisualize.h"
#include "mjrender.h"


// macros
#define mjMARKSTACK   int _mark = d->pstack;
#define mjFREESTACK   d->pstack = _mark;
#define mjDISABLED(x) (m->opt.disableflags & (x))
#define mjENABLED(x)  (m->opt.enableflags & (x))


// user error and memory handlers
MJAPI extern void  (*mju_user_error)(const char*);
MJAPI extern void  (*mju_user_warning)(const char*);
MJAPI extern void* (*mju_user_malloc)(size_t);
MJAPI extern void  (*mju_user_free)(void*);


// callbacks extending computation pipeline
MJAPI extern mjfGeneric  mjcb_endstep;
MJAPI extern mjfGeneric  mjcb_passive;
MJAPI extern mjfGeneric  mjcb_control;
MJAPI extern mjfTime     mjcb_time;
MJAPI extern mjfAct      mjcb_act_dyn;
MJAPI extern mjfAct      mjcb_act_gain;
MJAPI extern mjfAct      mjcb_act_bias;
MJAPI extern mjfMagnetic mjcb_magnetic;
MJAPI extern mjfSolImp   mjcb_sol_imp;
MJAPI extern mjfSolRef   mjcb_sol_ref;

// collision function table
MJAPI extern mjfCollision mjCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES];


// string names
MJAPI extern const char* mjVISSTRING[mjNVISFLAG][3];
MJAPI extern const char* mjRNDSTRING[mjNRNDFLAG][3];
MJAPI extern const char* mjDISABLESTRING[mjNDISABLE];
MJAPI extern const char* mjENABLESTRING[mjNENABLE];
MJAPI extern const char* mjTIMERSTRING[mjNTIMER];


//---------------------- License activation and certificate (mutex-protected) -----------

// activate license, call mju_error on failure; return 1 if ok, 0 if failure
MJAPI int mj_activate(const char* filename);

// deactivate license, free memory
MJAPI void mj_deactivate(void);

// server: generate certificate question
MJAPI void mj_certQuestion(mjtNum question[16]);

// client: generate certificate answer given question
MJAPI void mj_certAnswer(const mjtNum question[16], mjtNum answer[16]);

// server: check certificate question-answer pair; return 1 if match, 0 if mismatch
MJAPI int mj_certCheck(const mjtNum question[16], const mjtNum answer[16]);


//---------------------- XML parser and C++ compiler (mutex-protected) ------------------

// parse XML file or string in MJCF or URDF format, compile it, return low-level model
//  if xmlstring is not NULL, it has precedence over filename
//  error can be NULL; otherwise assumed to have size error_sz
MJAPI mjModel* mj_loadXML(const char* filename, const char* xmlstring, 
                          char* error, int error_sz);

// update XML data structures with info from low-level model, save as MJCF
//  error can be NULL; otherwise assumed to have size error_sz
MJAPI int mj_saveXML(const char* filename, const mjModel* m, char* error, int error_sz);

// print internal XML schema as plain text or HTML, with style-padding or &nbsp;
MJAPI int mj_printSchema(const char* filename, char* buffer, int buffer_sz, 
                         int flg_html, int flg_pad);


//---------------------- Main entry points ----------------------------------------------

// advance simulation: use control callback, no external force, RK4 available
MJAPI void mj_step(const mjModel* m, mjData* d);

// advance simulation in two steps: before external force/control is set by user
MJAPI void mj_step1(const mjModel* m, mjData* d);

// advance simulation in two steps: after external force/control is set by user
MJAPI void mj_step2(const mjModel* m, mjData* d);

// forward dynamics
MJAPI void mj_forward(const mjModel* m, mjData* d);

// inverse dynamics
MJAPI void mj_inverse(const mjModel* m, mjData* d);

// forward dynamics with skip: 0- no skip, 1- skip pos, 2- skip pos,vel
MJAPI void mj_forwardSkip(const mjModel* m, mjData* d, int skip);

// inverse dynamics with skip: 0- no skip, 1- skip pos, 2- skip pos,vel
MJAPI void mj_inverseSkip(const mjModel* m, mjData* d, int skip);

// sensor data
MJAPI void mj_sensor(const mjModel* m, mjData* d);

// energy
MJAPI void mj_energy(const mjModel* m, mjData* d);


//---------------------- Model and data initialization ----------------------------------

// set default solver paramters
MJAPI void mj_defaultSolRefImp(mjtNum* solref, mjtNum* solimp);

// set physics options to default values
MJAPI void mj_defaultOption(mjOption* opt);

// set visual options to default values
MJAPI void mj_defaultVisual(mjVisual* vis);

// copy mjModel; allocate new if dest is NULL
MJAPI mjModel* mj_copyModel(mjModel* dest, const mjModel* src);

// save model to binary file or memory buffer (buffer has precedence if szbuf>0)
MJAPI void mj_saveModel(const mjModel* m, const char* filename, void* buffer, int buffer_sz);

// load model from binary file or memory buffer (buffer has precedence if szbuf>0)
MJAPI mjModel* mj_loadModel(const char* filename, void* buffer, int buffer_sz);

// de-allocate model
MJAPI void mj_deleteModel(mjModel* m);

// size of buffer needed to hold model
MJAPI int mj_sizeModel(const mjModel* m);

// allocate mjData correponding to given model
MJAPI mjData* mj_makeData(const mjModel* m);

// copy mjData
MJAPI mjData* mj_copyData(mjData* dest, const mjModel* m, const mjData* src);

// set data to defaults
MJAPI void mj_resetData(const mjModel* m, mjData* d);

// set data to defaults, fill everything else with debug_value
MJAPI void mj_resetDataDebug(const mjModel* m, mjData* d, unsigned char debug_value);

// mjData stack allocate
MJAPI mjtNum* mj_stackAlloc(mjData* d, int size);

// de-allocate data
MJAPI void mj_deleteData(mjData* d);

// reset callbacks to defaults
MJAPI void mj_resetCallbacks(void);

// set constant fields of mjModel
MJAPI void mj_setConst(mjModel* m, mjData* d, int flg_actrange);


//---------------------- Printing -------------------------------------------------------

// print model to text file
MJAPI void mj_printModel(const mjModel* m, const char* filename); 

// print data to text file
MJAPI void mj_printData(const mjModel* m, mjData* d, const char* filename); 

// print matrix to screen
MJAPI void mju_printMat(const mjtNum* mat, int nr, int nc);


//---------------------- Components: forward dynamics -----------------------------------

// position-dependent computations
MJAPI void mj_fwdPosition(const mjModel* m, mjData* d);

// velocity-dependent computations
MJAPI void mj_fwdVelocity(const mjModel* m, mjData* d);

// compute actuator force
MJAPI void mj_fwdActuation(const mjModel* m, mjData* d);

// add up all non-constraint forces, compute qacc_unc
MJAPI void mj_fwdAcceleration(const mjModel* m, mjData* d);

// constraint solver
MJAPI void mj_fwdConstraint(const mjModel* m, mjData* d);

// Euler integrator, semi-implicit in velocity
MJAPI void mj_Euler(const mjModel* m, mjData* d);

// Runge-Kutta explicit order-N integrator
MJAPI void mj_RungeKutta(const mjModel* m, mjData* d, int N);


//---------------------- Components: inverse dynamics -----------------------------------

// position-dependent computations
MJAPI void mj_invPosition(const mjModel* m, mjData* d);

// velocity-dependent computations
MJAPI void mj_invVelocity(const mjModel* m, mjData* d);

// constraint solver
MJAPI void mj_invConstraint(const mjModel* m, mjData* d);

// compare forward and inverse dynamics, without changing results of forward dynamics
MJAPI void mj_compareFwdInv(const mjModel* m, mjData* d);


//---------------------- Sub-components of the computation pipeline ---------------------

// check positions; reset if bad
MJAPI void mj_checkPos(const mjModel* m, mjData* d);

// check velocities; reset if bad
MJAPI void mj_checkVel(const mjModel* m, mjData* d);

// check accelerations; reset if bad
MJAPI void mj_checkAcc(const mjModel* m, mjData* d);

// forward kinematics
MJAPI void mj_kinematics(const mjModel* m, mjData* d);

// map inertias and motion dofs to global frame centered at CoM
MJAPI void mj_comPos(const mjModel* m, mjData* d);

// compute camera and light positions and orientations
MJAPI void mj_camlight(const mjModel* m, mjData* d);

// compute tendon lengths, velocities and moment arms
MJAPI void mj_tendon(const mjModel* m, mjData* d);

// compute actuator transmission lengths and moments
MJAPI void mj_transmission(const mjModel* m, mjData* d);

// composite rigid body inertia algorithm
MJAPI void mj_crb(const mjModel* m, mjData* d);

// sparse L'*D*L factorizaton of the inertia matrix
MJAPI void mj_factorM(const mjModel* m, mjData* d);

// sparse backsubstitution:  x = inv(L'*D*L)*y
MJAPI void mj_backsubM(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n);

// half of sparse backsubstitution:  x = sqrt(inv(D))*inv(L')*y
MJAPI void mj_backsubM2(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n);

// compute cvel, cdof_dot
MJAPI void mj_comVel(const mjModel* m, mjData* d);

// spring-dampers and body viscosity
MJAPI void mj_passive(const mjModel* m, mjData* d);

// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term
MJAPI void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result);

// RNE with complete data: compute cacc, cfrc_ext, cfrc_int
MJAPI void mj_rnePostConstraint(const mjModel* m, mjData* d);

// collision detection
MJAPI void mj_collision(const mjModel* m, mjData* d);

// construct constraints
MJAPI void mj_makeConstraint(const mjModel* m, mjData* d);

// compute dense matrices: efc_AR, e_ARchol, fc_half, fc_AR
MJAPI void mj_projectConstraint(const mjModel* m, mjData* d);

// compute efc_vel, efc_aref
MJAPI void mj_referenceConstraint(const mjModel* m, mjData* d);


//---------------------- Support functions ----------------------------------------------

// determine type of friction cone
MJAPI int mj_isPyramid(const mjModel* m);

// determine type of constraint Jacobian
MJAPI int mj_isSparse(const mjModel* m);

// multiply Jacobian by vector
MJAPI void mj_mulJacVec(const mjModel* m, mjData* d, 
                        mjtNum* res, const mjtNum* vec);

// multiply JacobianT by vector
MJAPI void mj_mulJacTVec(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec);

// compute 3/6-by-nv Jacobian of global point attached to given body
MJAPI void mj_jac(const mjModel* m, const mjData* d, 
                  mjtNum* jacp, mjtNum* jacr, const mjtNum* point, int body);

// compute body frame Jacobian
MJAPI void mj_jacBody(const mjModel* m, const mjData* d, 
                      mjtNum* jacp, mjtNum* jacr, int body);

// compute body center-of-mass Jacobian
MJAPI void mj_jacBodyCom(const mjModel* m, const mjData* d, 
                         mjtNum* jacp, mjtNum* jacr, int body);

// compute geom Jacobian
MJAPI void mj_jacGeom(const mjModel* m, const mjData* d, 
                      mjtNum* jacp, mjtNum* jacr, int geom);

// compute site Jacobian
MJAPI void mj_jacSite(const mjModel* m, const mjData* d, 
                      mjtNum* jacp, mjtNum* jacr, int site);

// compute translation Jacobian of point, and rotation Jacobian of axis
MJAPI void mj_jacPointAxis(const mjModel* m, mjData* d, 
                           mjtNum* jacPoint, mjtNum* jacAxis, 
                           const mjtNum* point, const mjtNum* axis, int body);

// get id of object with specified name; -1: not found; type is mjtObj
MJAPI int mj_name2id(const mjModel* m, int type, const char* name);

// get name of object with specified id; 0: invalid type or id; type is mjtObj
MJAPI const char* mj_id2name(const mjModel* m, int type, int id);

// convert sparse inertia matrix M into full matrix
MJAPI void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M);

// multiply vector by inertia matrix
MJAPI void mj_mulM(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

// apply cartesian force and torque (outside xfrc_applied mechanism)
MJAPI void mj_applyFT(const mjModel* m, mjData* d, 
                      const mjtNum* force, const mjtNum* torque, 
                      const mjtNum* point, int body, mjtNum* qfrc_target);

// compute object 6D velocity in object-centered frame, world/local orientation
MJAPI void mj_objectVelocity(const mjModel* m, const mjData* d, 
                             int objtype, int objid, mjtNum* res, int flg_local);

// compute object 6D acceleration in object-centered frame, world/local orientation
MJAPI void mj_objectAcceleration(const mjModel* m, const mjData* d, 
                                 int objtype, int objid, mjtNum* res, int flg_local);

// compute velocity by finite-differencing two positions
MJAPI void mj_differentiatePos(const mjModel* m, mjtNum* qvel, mjtNum dt,
                               const mjtNum* qpos1, const mjtNum* qpos2);

// extract 6D force:torque for one contact, in contact frame
MJAPI void mj_contactForce(const mjModel* m, const mjData* d, int id, mjtNum* result);

// integrate position with given velocity
MJAPI void mj_integratePos(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt);

// normalize all quaterions in qpos-type vector
MJAPI void mj_normalizeQuat(const mjModel* m, mjtNum* qpos);

// map from body local to global Cartesian coordinates
MJAPI void mj_local2Global(mjData* d, mjtNum* xpos, mjtNum* xmat, 
                           const mjtNum* pos, const mjtNum* quat, int body);

// sum all body masses
MJAPI mjtNum mj_getTotalmass(const mjModel* m);

// scale body masses and inertias to achieve specified total mass
MJAPI void mj_setTotalmass(mjModel* m, mjtNum newmass);

// version number: 1.0.2 is encoded as 102
MJAPI int mj_version(void);


//---------------------- Asbtract visualization: 3D --------------------------------------

// init visualization objects, allocate buffers
MJAPI void mjv_makeObjects(mjvObjects* obj, int maxgeom);

// free visualization objects
MJAPI void mjv_freeObjects(mjvObjects* obj);

// set default visualization options
MJAPI void mjv_defaultOption(mjvOption* vopt);

// set default camera pose
MJAPI void mjv_defaultCameraPose(mjvCameraPose* pose);

// set default camera
MJAPI void mjv_defaultCamera(mjvCamera* cam);

// set high-level camera info, or pose for fixed camera
MJAPI void mjv_setCamera(const mjModel* m, const mjData* d, mjvCamera* cam);

// update camera pose given high-level info
MJAPI void mjv_updateCameraPose(mjvCamera* cam, mjtNum aspect);

// convert 3D vector to z-aligned world coordinates
MJAPI void mjv_convert3D(mjtNum* res, const mjtNum* vec,
                         mjtNum scale, const mjvCameraPose* campose);

// convert 2D mouse motion to z-aligned 3D world coordinates; mode is mjtMouse
MJAPI void mjv_convert2D(mjtNum* res, int mode, mjtNum dx, mjtNum dy,
                         mjtNum scale, const mjvCameraPose* campose);

// move camera; action is mjtMouse
MJAPI void mjv_moveCamera(int action, float dx, float dy, mjvCamera* cam, 
                          float width, float height);

// translate or rotate object; action is mjtMouse
MJAPI void mjv_moveObject(int action, float dx, float dy, const mjvCameraPose* campose,
                          float width, float height, mjtNum* pos, mjtNum* quat);

// compute mouse perturbation: result = (3D force, 3D torque)
MJAPI void mjv_mousePerturb(const mjModel* m, mjData* d, int select, int perturb, 
                            const mjtNum* refpos, const mjtNum* refquat, mjtNum* result);

// move selected subtree or fixed body
MJAPI void mjv_mouseEdit(mjModel* m, mjData* d, int select,
                         int perturb, const mjtNum* refpos, const mjtNum* refquat);

// make list of abstract geoms in mjvObjects
MJAPI void mjv_makeGeoms(const mjModel* m, mjData* d, mjvObjects* obj,
                         const mjvOption* vopt, int catmask, int select, 
                         const mjtNum* refpos, const mjtNum* refquat, const mjtNum* localpos);

// make list of abstract lights in mjvObjects
MJAPI void mjv_makeLights(const mjModel* m, mjData* d, mjvObjects* obj);


//---------------------- OpenGL rendering: 3D -------------------------------------------

// text overlay; gridpos is mjtGridPos
MJAPI void mjr_overlay(mjrRect viewport, int gridpos, int big,
                       const char* overlay, const char* overlay2, const mjrContext* con);

// draw rectangle
MJAPI void mjr_rectangle(mjrRect viewport,
                         double left, double bottom, double rwidth, double rheight,
                         double r, double g, double b, double a);

// plot 2d lines
MJAPI void mjr_lines(mjrRect viewport, int nline, const int* npoint, const mjtNum* data);

// call glFinish
MJAPI void mjr_finish(void);

// render text (normal or big)
MJAPI void mjr_text(const char* txt, const mjrContext* con, int big,
                    float x, float y, float z, float r, float g, float b);

// render text with background clear (always normal)
MJAPI void mjr_textback(const char* txt, const mjrContext* con,
                        float x, float y, float z, float r, float g, float b);

// compute text width (normal or big)
MJAPI int mjr_textWidth(const char* txt, const mjrContext* con, int big);

// set default mjrOption
MJAPI void mjr_defaultOption(mjrOption* ropt);

// set default mjrContext
MJAPI void mjr_defaultContext(mjrContext* con);

// (re) upload texture to GPU
MJAPI void mjr_uploadTexture(const mjModel* m, const mjrContext* con, int texid);

// (re) upload mesh to GPU
MJAPI void mjr_uploadMesh(const mjModel* m, const mjrContext* con, int meshid);

// (re) upload height field to GPU
MJAPI void mjr_uploadHField(const mjModel* m, const mjrContext* con, int hfieldid);

// allocate resources in custom OpenGL context; fontscale = 100, 150, 200 (%)
MJAPI void mjr_makeContext(const mjModel* m, mjrContext* con, int fontscale);

// free resources in custom OpenGL context
MJAPI void mjr_freeContext(mjrContext* con);

// 3D rendering
MJAPI void mjr_render(int flg_offscreen, mjrRect viewport, mjvObjects* obj,
                      const mjrOption* ropt, mjvCameraPose* campose, const mjrContext* con);

// 3D selection
MJAPI int mjr_select(mjrRect viewport, mjvObjects* obj, 
                     int mousex, int mousey, mjtNum* pos, mjtNum* depth,
                     const mjrOption* ropt, mjvCameraPose* campose, const mjrContext* con);

// show offscreen image
MJAPI void mjr_showOffscreen(int left, int bottom, const mjrContext* con);

// show image from RGB buffer
MJAPI void mjr_showBuffer(unsigned char* rgb, int rgbwidth, int rgbheight,
                          int left, int bottom, const mjrContext* con);

// get offscreen image
MJAPI void mjr_getOffscreen(unsigned char* rgb, float* depth,
                            mjrRect viewport, const mjrContext* con);

// get backbuffer image
MJAPI void mjr_getBackbuffer(unsigned char* rgb, float* depth,
                             mjrRect viewport, const mjrContext* con);

// call glGetError internally and return result
MJAPI int mjr_getError(void);


//---------------------- Utility functions: error and memory ----------------------------

// main error function; does not return to caller
MJAPI void mju_error(const char* msg);

// error function with int argument; msg is a printf format string
MJAPI void mju_error_i(const char* msg, int i);

// error function with string argument
MJAPI void mju_error_s(const char* msg, const char* text);

// main warning function; returns to caller
MJAPI void mju_warning(const char* msg);

// warning function with int argument
MJAPI void mju_warning_i(const char* msg, int i);

// warning function with string argument
MJAPI void mju_warning_s(const char* msg, const char* text);

// clear user error and memory handlers
MJAPI void mju_clearHandlers(void);

// allocate memory; byte-align on 8; pad size to multiple of 8
MJAPI void* mju_malloc(size_t size);

// free memory (with free() by default)
MJAPI void mju_free(void* ptr);

// high-level warning function: count warnings in mjData, print only the first
MJAPI void mj_warning(mjData* d, int warning, int info);


//---------------------- Utility functions: basic math ----------------------------------

#define mjMAX(a,b) (((a) > (b)) ? (a) : (b))
#define mjMIN(a,b) (((a) < (b)) ? (a) : (b))

#ifdef mjUSEDOUBLE
    #define mju_sqrt    sqrt
    #define mju_exp     exp
    #define mju_sin     sin
    #define mju_cos     cos
    #define mju_tan     tan
    #define mju_asin    asin
    #define mju_acos    acos
    #define mju_atan2   atan2
    #define mju_tanh    tanh
    #define mju_pow     pow
    #define mju_abs     fabs
    #define mju_log     log
    #define mju_log10   log10
    #define mju_floor   floor
    #define mju_ceil    ceil

#else
    #define mju_sqrt    sqrtf
    #define mju_exp     expf
    #define mju_sin     sinf
    #define mju_cos     cosf
    #define mju_tan     tanf
    #define mju_asin    asinf
    #define mju_acos    acosf
    #define mju_atan2   atan2f
    #define mju_tanh    tanhf
    #define mju_pow     powf
    #define mju_abs     fabsf
    #define mju_log     logf
    #define mju_log10   log10f
    #define mju_floor   floorf
    #define mju_ceil    ceilf
#endif

// set vector to zero
MJAPI void mju_zero3(mjtNum* res);

// copy vector
MJAPI void mju_copy3(mjtNum* res, const mjtNum* data);

// scale vector
MJAPI void mju_scl3(mjtNum* res, const mjtNum* vec, mjtNum scl);

// add vectors
MJAPI void mju_add3(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2);

// subtract vectors
MJAPI void mju_sub3(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2);

// add to vector
MJAPI void mju_addTo3(mjtNum* res, const mjtNum* vec);

// add scaled to vector
MJAPI void mju_addToScl3(mjtNum* res, const mjtNum* vec, mjtNum scl);

// res = vec1 + scl*vec2
MJAPI void mju_addScl3(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, mjtNum scl);

// normalize vector, return length before normalization
MJAPI mjtNum mju_normalize3(mjtNum* res);

// compute vector length (without normalizing)
MJAPI mjtNum mju_norm3(const mjtNum* res);

// vector dot-product
MJAPI mjtNum mju_dot3(const mjtNum* vec1, const mjtNum* vec2);

// Cartesian distance between 3D vectors
MJAPI mjtNum mju_dist3(const mjtNum* pos1, const mjtNum* pos2);

// multiply vector by 3D rotation matrix
MJAPI void mju_rotVecMat(mjtNum* res, const mjtNum* vec, const mjtNum* mat);

// multiply vector by transposed 3D rotation matrix
MJAPI void mju_rotVecMatT(mjtNum* res, const mjtNum* vec, const mjtNum* mat);

// vector cross-product, 3D
MJAPI void mju_cross(mjtNum* res, const mjtNum* a, const mjtNum* b);

// set vector to zero
MJAPI void mju_zero(mjtNum* res, int n);

// copy vector
MJAPI void mju_copy(mjtNum* res, const mjtNum* data, int n);

// scale vector
MJAPI void mju_scl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n);

// add vectors
MJAPI void mju_add(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n);

// subtract vectors
MJAPI void mju_sub(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n);

// add to vector
MJAPI void mju_addTo(mjtNum* res, const mjtNum* vec, int n);

// add scaled to vector
MJAPI void mju_addToScl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n);

// res = vec1 + scl*vec2
MJAPI void mju_addScl(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, 
                      mjtNum scl, int n);

// normalize vector, return length before normalization
MJAPI mjtNum mju_normalize(mjtNum* res, int n);

// compute vector length (without normalizing)
MJAPI mjtNum mju_norm(const mjtNum* res, int n);

// vector dot-product
MJAPI mjtNum mju_dot(const mjtNum* vec1, const mjtNum* vec2, const int n);

// multiply matrix and vector
MJAPI void mju_mulMatVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                         int nr, int nc);

// multiply transposed matrix and vector
MJAPI void mju_mulMatTVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                          int nr, int nc);

// transpose matrix
MJAPI void mju_transpose(mjtNum* res, const mjtNum* mat, int r, int c);

// multiply matrices
MJAPI void mju_mulMatMat(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                         int r1, int c1, int c2);

// multiply matrices, second argument transposed
MJAPI void mju_mulMatMatT(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                          int r1, int c1, int r2);

// multiply matrices, first argument transposed
MJAPI void mju_mulMatTMat(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                          int r1, int c1, int c2);

// compute M*M'; scratch must be at least r*c
MJAPI void mju_sqrMat(mjtNum* res, const mjtNum* mat, int r, int c, 
                      mjtNum* scratch, int nscratch);

// compute M'*diag*M (diag=NULL: compute M'*M)
MJAPI void mju_sqrMatTD(mjtNum* res, const mjtNum* mat, const mjtNum* diag, int r, int c);

// coordinate transform of 6D motion or force vector in rotation:translation format
//  rotnew2old is 3-by-3, NULL means no rotation; flg_force specifies force or motion type
MJAPI void mju_transformSpatial(mjtNum* res, const mjtNum* vec, int flg_force,
                                const mjtNum* newpos, const mjtNum* oldpos, 
                                const mjtNum* rotnew2old);


//---------------------- Utility functions: quaternions ---------------------------------

// rotate vector by quaternion
MJAPI void mju_rotVecQuat(mjtNum* res, const mjtNum* vec, const mjtNum* quat);

// negate quaternion
MJAPI void mju_negQuat(mjtNum* res, const mjtNum* quat);

// muiltiply quaternions
MJAPI void mju_mulQuat(mjtNum* res, const mjtNum* quat1, const mjtNum* quat2);

// muiltiply quaternion and axis
MJAPI void mju_mulQuatAxis(mjtNum* res, const mjtNum* quat, const mjtNum* axis);

// convert axisAngle to quaternion
MJAPI void mju_axisAngle2Quat(mjtNum* res, const mjtNum* axis, mjtNum angle);

// convert quaternion (corresponding to orientation difference) to 3D velocity
MJAPI void mju_quat2Vel(mjtNum* res, const mjtNum* quat, mjtNum dt);

// convert quaternion to 3D rotation matrix
MJAPI void mju_quat2Mat(mjtNum* res, const mjtNum* quat);

// convert 3D rotation matrix to quaterion
MJAPI void mju_mat2Quat(mjtNum* quat, const mjtNum* mat);

// time-derivative of quaternion, given 3D rotational velocity
MJAPI void mju_derivQuat(mjtNum* res, const mjtNum* quat, const mjtNum* vel);

// integrate quaterion given 3D angular velocity
MJAPI void mju_quatIntegrate(mjtNum* quat, const mjtNum* vel, mjtNum scale);

// compute quaternion performing rotation from given vector to z-axis
MJAPI void mju_quatVec2Z(mjtNum* quat, const mjtNum* vec);


//---------------------- Utility functions: matrix decomposition ------------------------

// Cholesky decomposition
MJAPI int mju_cholFactor(mjtNum* mat, mjtNum* diag, int n,
                         mjtNum minabs, mjtNum minrel, mjtNum* correct);

// Cholesky backsubstitution: phase&i enables forward(i=1), backward(i=2) pass
MJAPI void mju_cholBacksub(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                           int n, int nvec, int phase);

// eigenvalue decomposition of symmetric 3x3 matrix
MJAPI int mju_eig3(mjtNum* eigval, mjtNum* eigvec, mjtNum* quat, const mjtNum* mat);


//---------------------- Utility functions: miscellaneous -------------------------------

// muscle FVL curve: prm = (lminrel, lmaxrel, widthrel, vmaxrel, fmax, fvsat)
MJAPI mjtNum mju_muscleFVL(mjtNum len, mjtNum vel, mjtNum lmin, mjtNum lmax, mjtNum* prm);

// muscle passive force: prm = (lminrel, lmaxrel, fpassive)
MJAPI mjtNum mju_musclePassive(mjtNum len, mjtNum lmin, mjtNum lmax, mjtNum* prm);

// pneumatic cylinder dynamics
MJAPI mjtNum mju_pneumatic(mjtNum len, mjtNum len0, mjtNum vel, mjtNum* prm,
                           mjtNum act, mjtNum ctrl, mjtNum timestep, mjtNum* jac);

// convert contact force to pyramid representation
MJAPI void mju_encodePyramid(mjtNum* pyramid, const mjtNum* force,
                             const mjtNum* mu, int dim);

// convert pyramid representation to contact force
MJAPI void mju_decodePyramid(mjtNum* force, const mjtNum* pyramid,
                             const mjtNum* mu, int dim);

// integrate spring-damper analytically, return pos(dt)
MJAPI mjtNum mju_springDamper(mjtNum pos0, mjtNum vel0, mjtNum Kp, mjtNum Kv, mjtNum dt);

// min function, single evaluation of a and b
MJAPI mjtNum mju_min(mjtNum a, mjtNum b);

// max function, single evaluation of a and b
MJAPI mjtNum mju_max(mjtNum a, mjtNum b);

// sign function
MJAPI mjtNum mju_sign(mjtNum x);

// round to nearest integer
MJAPI int mju_round(mjtNum x);

// convert type id (mjtObj) to type name
MJAPI const char* mju_type2Str(int type);

// convert type name to type id (mjtObj)
MJAPI int mju_str2Type(const char* str);

// warning text
MJAPI const char* mju_warningText(int warning, int info);

// return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise
MJAPI int mju_isBad(mjtNum x);

// return 1 if all elements are 0
MJAPI int mju_isZero(mjtNum* vec, int n);


#if defined(__cplusplus)
}
#endif
