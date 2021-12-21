include "mjmodel.pxd"
include "mjdata.pxd"
include "mjrender.pxd"
include "mjui.pxd"
include "mjvisualize.pxd"


cdef extern from "mujoco.h" nogil:
    # macros
    #define mjMARKSTACK   int _mark = d->pstack;
    #define mjFREESTACK   d->pstack = _mark;
    #define mjDISABLED(x) (m->opt.disableflags & (x))
    #define mjENABLED(x)  (m->opt.enableflags & (x))


    # user error and memory handlers
    void  (*mju_user_error)(const char*);
    void  (*mju_user_warning)(const char*);
    void* (*mju_user_malloc)(size_t);
    void  (*mju_user_free)(void*);


    # # callbacks extending computation pipeline
    # mjfGeneric  mjcb_passive;
    # mjfGeneric  mjcb_control;
    # mjfSensor   mjcb_sensor;
    # mjfTime     mjcb_time;
    # mjfAct      mjcb_act_dyn;
    mjfAct      mjcb_act_gain;
    mjfAct      mjcb_act_bias;
    # mjfSolImp   mjcb_sol_imp;
    # mjfSolRef   mjcb_sol_ref;
    #
    #
    # # collision function table
    # mjfCollision mjCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES];
    #
    #
    # # string names
    const char* mjDISABLESTRING[mjNDISABLE];
    const char* mjENABLESTRING[mjNENABLE];
    const char* mjTIMERSTRING[mjNTIMER];
    const char* mjLABELSTRING[mjNLABEL];
    const char* mjFRAMESTRING[mjNFRAME];
    const char* mjVISSTRING[mjNVISFLAG][3];
    const char* mjRNDSTRING[mjNRNDFLAG][3];


    #---------------------- Activation -----------------------------------------------------

    # activate license, call mju_error on failure; return 1 if ok, 0 if failure
    int mj_activate(const char* filename);

    # deactivate license, free memory
    void mj_deactivate();

    #---------------------- Virtual file system --------------------------------------------

    # Initialize VFS to empty (no deallocation).
    void mj_defaultVFS(mjVFS* vfs);

    # Add file to VFS, return 0: success, 1: full, 2: repeated name, -1: not found on disk.
    int mj_addFileVFS(mjVFS* vfs, const char* directory, const char* filename);

    # Make empty file in VFS, return 0: success, 1: full, 2: repeated name.
    int mj_makeEmptyFileVFS(mjVFS* vfs, const char* filename, int filesize);

    # Return file index in VFS, or -1 if not found in VFS.
    int mj_findFileVFS(const mjVFS* vfs, const char* filename);

    # Delete file from VFS, return 0: success, -1: not found in VFS.
    int mj_deleteFileVFS(mjVFS* vfs, const char* filename);

    # Delete all files from VFS.
    void mj_deleteVFS(mjVFS* vfs);

    #--------------------- Parse and compile ----------------------------------------------

    # Parse XML file in MJCF or URDF format, compile it, return low-level model.
    # If vfs is not NULL, look up files in vfs before reading from disk.
    # If error is not NULL, it must have size error_sz.
    mjModel* mj_loadXML(const char* filename, const mjVFS* vfs,
                              char* error, int error_sz);

    # Update XML data structures with info from low-level model, save as MJCF.
    # If error is not NULL, it must have size error_sz.
    int mj_saveLastXML(const char* filename, const mjModel* m,
                             char* error, int error_sz);

    # Free last XML model if loaded. Called internally at each load.
    void mj_freeLastXML();

    # Print internal XML schema as plain text or HTML, with style-padding or &nbsp;.
    int mj_printSchema(const char* filename, char* buffer, int buffer_sz,
                             int flg_html, int flg_pad);


    #--------------------- Main simulation ------------------------------------------------

    # Advance simulation, use control callback to obtain external force and control.
    void mj_step(const mjModel* m, mjData* d);

    # Advance simulation in two steps: before external force and control is set by user.
    void mj_step1(const mjModel* m, mjData* d);

    # Advance simulation in two steps: after external force and control is set by user.
    void mj_step2(const mjModel* m, mjData* d);

    # Forward dynamics: same as mj_step but do not integrate in time.
    void mj_forward(const mjModel* m, mjData* d);

    # Inverse dynamics: qacc must be set before calling.
    void mj_inverse(const mjModel* m, mjData* d);

    # Forward dynamics with skip; skipstage is mjtStage.
    void mj_forwardSkip(const mjModel* m, mjData* d,
                              int skipstage, int skipsensorenergy);

    # Inverse dynamics with skip; skipstage is mjtStage.
    void mj_inverseSkip(const mjModel* m, mjData* d,
                        int skipstage, int skipsensorenergy);

    # Forward dynamics with skip; skipstage is mjtStage.
    void mj_forwardSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor);

    # Inverse dynamics with skip; skipstage is mjtStage.
    void mj_inverseSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor);


    #--------------------- Initialization -------------------------------------------------

    # Set default options for length range computation.
    void mj_defaultLROpt(mjLROpt* opt);

    # Set solver parameters to default values.
    void mj_defaultSolRefImp(mjtNum* solref, mjtNum* solimp);

    # Set physics options to default values.
    void mj_defaultOption(mjOption* opt);

    # Set visual options to default values.
    void mj_defaultVisual(mjVisual* vis);

    # Copy mjModel, allocate new if dest is NULL.
    mjModel* mj_copyModel(mjModel* dest, const mjModel* src);

    # Save model to binary MJB file or memory buffer; buffer has precedence when given.
    void mj_saveModel(const mjModel* m, const char* filename, void* buffer, int buffer_sz);

    # Load model from binary MJB file.
    # If vfs is not NULL, look up file in vfs before reading from disk.
    mjModel* mj_loadModel(const char* filename, mjVFS* vfs);

    # Free memory allocation in model.
    void mj_deleteModel(mjModel* m);

    # Return size of buffer needed to hold model.
    int mj_sizeModel(const mjModel* m);

    # Allocate mjData correponding to given model.
    mjData* mj_makeData(const mjModel* m);

    # Copy mjData.
    mjData* mj_copyData(mjData* dest, const mjModel* m, const mjData* src);

    # Reset data to defaults.
    void mj_resetData(const mjModel* m, mjData* d);

    # Reset data to defaults, fill everything else with debug_value.
    void mj_resetDataDebug(const mjModel* m, mjData* d, unsigned char debug_value);

    # Reset data, set fields from specified keyframe.
    void mj_resetDataKeyframe(const mjModel* m, mjData* d, int key);

    # Allocate array of specified size on mjData stack. Call mju_error on stack overflow.
    mjtNum* mj_stackAlloc(mjData* d, int size);

    # Free memory allocation in mjData.
    void mj_deleteData(mjData* d);

    # Reset all callbacks to NULL pointers (NULL is the default).
    void mj_resetCallbacks();

    # Set constant fields of mjModel, corresponding to qpos0 configuration.
    void mj_setConst(mjModel* m, mjData* d);

    # Set actuator_lengthrange for specified actuator; return 1 if ok, 0 if error.
    int mj_setLengthRange(mjModel* m, mjData* d, int index,
                          const mjLROpt* opt, char* error, int error_sz);

    #--------------------- Printing -------------------------------------------------------

    # Print model to text file.
    void mj_printModel(const mjModel* m, const char* filename);

    # Print data to text file.
    void mj_printData(const mjModel* m, mjData* d, const char* filename);

    # Print matrix to screen.
    void mju_printMat(const mjtNum* mat, int nr, int nc);

    # Print sparse matrix to screen.
    void mju_printMatSparse(const mjtNum* mat, int nr,
                                  const int* rownnz, const int* rowadr,
                                  const int* colind);


    #--------------------- Components -----------------------------------------------------

    # Run position-dependent computations.
    void mj_fwdPosition(const mjModel* m, mjData* d);

    # Run velocity-dependent computations.
    void mj_fwdVelocity(const mjModel* m, mjData* d);

    # Compute actuator force qfrc_actuation.
    void mj_fwdActuation(const mjModel* m, mjData* d);

    # Add up all non-constraint forces, compute qacc_unc.
    void mj_fwdAcceleration(const mjModel* m, mjData* d);

    # Run selected constraint solver.
    void mj_fwdConstraint(const mjModel* m, mjData* d);

    # Euler integrator, semi-implicit in velocity.
    void mj_Euler(const mjModel* m, mjData* d);

    # Runge-Kutta explicit order-N integrator.
    void mj_RungeKutta(const mjModel* m, mjData* d, int N);

    # Run position-dependent computations in inverse dynamics.
    void mj_invPosition(const mjModel* m, mjData* d);

    # Run velocity-dependent computations in inverse dynamics.
    void mj_invVelocity(const mjModel* m, mjData* d);

    # Apply the analytical formula for inverse constraint dynamics.
    void mj_invConstraint(const mjModel* m, mjData* d);

    # Compare forward and inverse dynamics, save results in fwdinv.
    void mj_compareFwdInv(const mjModel* m, mjData* d);


    #--------------------- Sub components -------------------------------------------------

    # Evaluate position-dependent sensors.
    void mj_sensorPos(const mjModel* m, mjData* d);

    # Evaluate velocity-dependent sensors.
    void mj_sensorVel(const mjModel* m, mjData* d);

    # Evaluate acceleration and force-dependent sensors.
    void mj_sensorAcc(const mjModel* m, mjData* d);

    # Evaluate position-dependent energy (potential).
    void mj_energyPos(const mjModel* m, mjData* d);

    # Evaluate velocity-dependent energy (kinetic).
    void mj_energyVel(const mjModel* m, mjData* d);

    # Check qpos, reset if any element is too big or nan.
    void mj_checkPos(const mjModel* m, mjData* d);

    # Check qvel, reset if any element is too big or nan.
    void mj_checkVel(const mjModel* m, mjData* d);

    # Check qacc, reset if any element is too big or nan.
    void mj_checkAcc(const mjModel* m, mjData* d);

    # Run forward kinematics.
    void mj_kinematics(const mjModel* m, mjData* d);

    # Map inertias and motion dofs to global frame centered at CoM.
    void mj_comPos(const mjModel* m, mjData* d);

    # Compute camera and light positions and orientations.
    void mj_camlight(const mjModel* m, mjData* d);

    # Compute tendon lengths, velocities and moment arms.
    void mj_tendon(const mjModel* m, mjData* d);

    # Compute actuator transmission lengths and moments.
    void mj_transmission(const mjModel* m, mjData* d);

    # Run composite rigid body inertia algorithm (CRB).
    void mj_crb(const mjModel* m, mjData* d);

    # Compute sparse L'*D*L factorizaton of inertia matrix.
    void mj_factorM(const mjModel* m, mjData* d);

    # Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y
    void mj_solveM(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n);

    # Half of linear solve:  x = sqrt(inv(D))*inv(L')*y
    void mj_solveM2(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n);

    # Compute cvel, cdof_dot.
    void mj_comVel(const mjModel* m, mjData* d);

    # Compute qfrc_passive from spring-dampers, viscosity and density.
    void mj_passive(const mjModel* m, mjData* d);

    # subtree linear velocity and angular momentum
    void mj_subtreeVel(const mjModel* m, mjData* d);

    # RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term.
    void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result);

    # RNE with complete data: compute cacc, cfrc_ext, cfrc_int.
    void mj_rnePostConstraint(const mjModel* m, mjData* d);

    # Run collision detection.
    void mj_collision(const mjModel* m, mjData* d);

    # Construct constraints.
    void mj_makeConstraint(const mjModel* m, mjData* d);

    # Compute inverse constaint inertia efc_AR.
    void mj_projectConstraint(const mjModel* m, mjData* d);

    # Compute efc_vel, efc_aref.
    void mj_referenceConstraint(const mjModel* m, mjData* d);

    # Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians.
    # If cost is not NULL, set *cost = s(jar) where jar = Jac*qacc-aref.
    void mj_constraintUpdate(const mjModel* m, mjData* d, const mjtNum* jar,
                                   mjtNum* cost, int flg_coneHessian);


    #--------------------- Support --------------------------------------------------------

    # Add contact to d->contact list; return 0 if success; 1 if buffer full.
    int mj_addContact(const mjModel* m, mjData* d, const mjContact* con);

    # Determine type of friction cone.
    int mj_isPyramidal(const mjModel* m);

    # Determine type of constraint Jacobian.
    int mj_isSparse(const mjModel* m);

    # Determine type of solver (PGS is dual, CG and Newton are primal).
    int mj_isDual(const mjModel* m);

    # Multiply dense or sparse constraint Jacobian by vector.
    void mj_mulJacVec(const mjModel* m, mjData* d,
                            mjtNum* res, const mjtNum* vec);

    # Multiply dense or sparse constraint Jacobian transpose by vector.
    void mj_mulJacTVec(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec);

    # Compute 3/6-by-nv end-effector Jacobian of global point attached to given body.
    void mj_jac(const mjModel* m, const mjData* d,
                      mjtNum* jacp, mjtNum* jacr, const mjtNum point[3], int body);

    # Compute body frame end-effector Jacobian.
    void mj_jacBody(const mjModel* m, const mjData* d,
                          mjtNum* jacp, mjtNum* jacr, int body);

    # Compute body center-of-mass end-effector Jacobian.
    void mj_jacBodyCom(const mjModel* m, const mjData* d,
                             mjtNum* jacp, mjtNum* jacr, int body);

    # Compute geom end-effector Jacobian.
    void mj_jacGeom(const mjModel* m, const mjData* d,
                          mjtNum* jacp, mjtNum* jacr, int geom);

    # Compute site end-effector Jacobian.
    void mj_jacSite(const mjModel* m, const mjData* d,
                          mjtNum* jacp, mjtNum* jacr, int site);

    # Compute translation end-effector Jacobian of point, and rotation Jacobian of axis.
    void mj_jacPointAxis(const mjModel* m, mjData* d,
                               mjtNum* jacPoint, mjtNum* jacAxis,
                               const mjtNum point[3], const mjtNum axis[3], int body);

    # Get id of object with specified name, return -1 if not found; type is mjtObj.
    int mj_name2id(const mjModel* m, int type, const char* name);

    # Get name of object with specified id, return 0 if invalid type or id; type is mjtObj.
    const char* mj_id2name(const mjModel* m, int type, int id);

    # Convert sparse inertia matrix M into full (i.e. dense) matrix.
    void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M);

    # Multiply vector by inertia matrix.
    void mj_mulM(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

    # Multiply vector by (inertia matrix)^(1/2).
    void mj_mulM2(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

    # Add inertia matrix to destination matrix.
    # Destination can be sparse uncompressed, or dense when all int* are NULL
    void mj_addM(const mjModel* m, mjData* d, mjtNum* dst,
                       int* rownnz, int* rowadr, int* colind);

    # Apply cartesian force and torque (outside xfrc_applied mechanism).
    void mj_applyFT(const mjModel* m, mjData* d,
                          const mjtNum* force, const mjtNum* torque,
                          const mjtNum* point, int body, mjtNum* qfrc_target);

    # Compute object 6D velocity in object-centered frame, world/local orientation.
    void mj_objectVelocity(const mjModel* m, const mjData* d,
                                 int objtype, int objid, mjtNum* res, int flg_local);

    # Compute object 6D acceleration in object-centered frame, world/local orientation.
    void mj_objectAcceleration(const mjModel* m, const mjData* d,
                                     int objtype, int objid, mjtNum* res, int flg_local);

    # Extract 6D force:torque for one contact, in contact frame.
    void mj_contactForce(const mjModel* m, const mjData* d, int id, mjtNum* result);

    # Compute velocity by finite-differencing two positions.
    void mj_differentiatePos(const mjModel* m, mjtNum* qvel, mjtNum dt,
                             const mjtNum* qpos1, const mjtNum* qpos2);

    # Integrate position with given velocity.
    void mj_integratePos(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt);

    # Normalize all quaterions in qpos-type vector.
    void mj_normalizeQuat(const mjModel* m, mjtNum* qpos);

    # Map from body local to global Cartesian coordinates.
    void mj_local2Global(mjData* d, mjtNum* xpos, mjtNum* xmat, const mjtNum* pos, const mjtNum* quat,
                         int body, mjtByte sameframe);

    # Sum all body masses.
    mjtNum mj_getTotalmass(const mjModel* m);

    # Scale body masses and inertias to achieve specified total mass.
    void mj_setTotalmass(mjModel* m, mjtNum newmass);

    # Return version number: 1.0.2 is encoded as 102.
    int mj_version();


    #--------------------- Ray collisions -------------------------------------------------

    # Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude.
    # Return geomid and distance (x) to nearest surface, or -1 if no intersection.
    # geomgroup, flg_static are as in mjvOption; geomgroup==NULL skips group exclusion.
    mjtNum mj_ray(const mjModel* m, const mjData* d, const mjtNum* pnt, const mjtNum* vec,
                        const mjtByte* geomgroup, mjtByte flg_static, int bodyexclude,
                        int* geomid);

    # Interect ray with hfield, return nearest distance or -1 if no intersection.
    mjtNum mj_rayHfield(const mjModel* m, const mjData* d, int geomid,
                              const mjtNum* pnt, const mjtNum* vec);

    # Interect ray with mesh, return nearest distance or -1 if no intersection.
    mjtNum mj_rayMesh(const mjModel* m, const mjData* d, int geomid,
                            const mjtNum* pnt, const mjtNum* vec);

    # Interect ray with pure geom, return nearest distance or -1 if no intersection.
    mjtNum mju_rayGeom(const mjtNum* pos, const mjtNum* mat, const mjtNum* size,
                             const mjtNum* pnt, const mjtNum* vec, int geomtype);


    #--------------------- Interaction ----------------------------------------------------

    # Set default camera.
    void mjv_defaultCamera(mjvCamera* cam);

    # Set default perturbation.
    void mjv_defaultPerturb(mjvPerturb* pert);

    # Transform pose from room to model space.
    void mjv_room2model(mjtNum* modelpos, mjtNum* modelquat, const mjtNum* roompos,
                              const mjtNum* roomquat, const mjvScene* scn);

    # Transform pose from model to room space.
    void mjv_model2room(mjtNum* roompos, mjtNum* roomquat, const mjtNum* modelpos,
                              const mjtNum* modelquat, const mjvScene* scn);

    # Get camera info in model space; average left and right OpenGL cameras.
    void mjv_cameraInModel(mjtNum* headpos, mjtNum* forward, mjtNum* up,
                                 const mjvScene* scn);

    # Get camera info in room space; average left and right OpenGL cameras.
    void mjv_cameraInRoom(mjtNum* headpos, mjtNum* forward, mjtNum* up,
                                const mjvScene* scn);

    # Get frustum height at unit distance from camera; average left and right OpenGL cameras.
    mjtNum mjv_frustumHeight(const mjvScene* scn);

    # Rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y).
    void mjv_alignToCamera(mjtNum* res, const mjtNum* vec, const mjtNum* forward);

    # Move camera with mouse; action is mjtMouse.
    void mjv_moveCamera(const mjModel* m, int action, mjtNum reldx, mjtNum reldy,
                              const mjvScene* scn, mjvCamera* cam);

    # Move perturb object with mouse; action is mjtMouse.
    void mjv_movePerturb(const mjModel* m, const mjData* d, int action, mjtNum reldx,
                               mjtNum reldy, const mjvScene* scn, mjvPerturb* pert);

    # Move model with mouse; action is mjtMouse.
    void mjv_moveModel(const mjModel* m, int action, mjtNum reldx, mjtNum reldy,
                             const mjtNum* roomup, mjvScene* scn);

    # Copy perturb pos,quat from selected body; set scale for perturbation.
    void mjv_initPerturb(const mjModel* m, const mjData* d,
                               const mjvScene* scn, mjvPerturb* pert);

    # Set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise.
    # Write d->qpos only if flg_paused and subtree root for selected body has free joint.
    void mjv_applyPerturbPose(const mjModel* m, mjData* d, const mjvPerturb* pert,
                                    int flg_paused);

    # Set perturb force,torque in d->xfrc_applied, if selected body is dynamic.
    void mjv_applyPerturbForce(const mjModel* m, mjData* d, const mjvPerturb* pert);

    # Return the average of two OpenGL cameras.
    mjvGLCamera mjv_averageCamera(const mjvGLCamera* cam1, const mjvGLCamera* cam2);

    # Select geom or skin with mouse, return bodyid; -1: none selected.
    int mjv_select(const mjModel* m, const mjData* d, const mjvOption* vopt,
                   mjtNum aspectratio, mjtNum relx, mjtNum rely,
                   const mjvScene* scn, mjtNum* selpnt, int* geomid, int* skinid);

    #--------------------- Visualization --------------------------------------------------

    # Set default visualization options.
    void mjv_defaultOption(mjvOption* opt);

    # Set default figure.
    void mjv_defaultFigure(mjvFigure* fig);

    # Initialize given geom fields when not NULL, set the rest to their default values.
    void mjv_initGeom(mjvGeom* geom, int type, const mjtNum* size,
                            const mjtNum* pos, const mjtNum* mat, const float* rgba);

    # Set (type, size, pos, mat) for connector-type geom between given points.
    # Assume that mjv_initGeom was already called to set all other properties.
    void mjv_makeConnector(mjvGeom* geom, int type, mjtNum width,
                                 mjtNum a0, mjtNum a1, mjtNum a2,
                                 mjtNum b0, mjtNum b1, mjtNum b2);

    # Set default abstract scene.
    void mjv_defaultScene(mjvScene* scn);

    # Allocate resources in abstract scene.
    void mjv_makeScene(const mjModel* m, mjvScene* scn, int maxgeom);

    # Free abstract scene.
    void mjv_freeScene(mjvScene* scn);

    # Update entire scene given model state.
    void mjv_updateScene(const mjModel* m, mjData* d, const mjvOption* opt,
                               const mjvPerturb* pert, mjvCamera* cam, int catmask, mjvScene* scn);

    # Add geoms from selected categories to existing scene.
    void mjv_addGeoms(const mjModel* m, mjData* d, const mjvOption* opt,
                            const mjvPerturb* pert, int catmask, mjvScene* scn);

    # Make list of lights.
    void mjv_makeLights(const mjModel* m, mjData* d, mjvScene* scn);

    # Update camera only.
    void mjv_updateCamera(const mjModel* m, mjData* d, mjvCamera* cam, mjvScene* scn);

    # Update skins.
    void mjv_updateSkin(const mjModel* m, mjData* d, mjvScene* scn);

    #--------------------- OpenGL rendering -----------------------------------------------

    # Set default mjrContext.
    void mjr_defaultContext(mjrContext* con);

    # Allocate resources in custom OpenGL context; fontscale is mjtFontScale.
    void mjr_makeContext(const mjModel* m, mjrContext* con, int fontscale);

    # Change font of existing context.
    void mjr_changeFont(int fontscale, mjrContext* con);

    # Add Aux buffer with given index to context; free previous Aux buffer.
    void mjr_addAux(int index, int width, int height, int samples, mjrContext* con);

    # Free resources in custom OpenGL context, set to default.
    void mjr_freeContext(mjrContext* con);

    # Upload texture to GPU, overwriting previous upload if any.
    void mjr_uploadTexture(const mjModel* m, const mjrContext* con, int texid);

    # Upload mesh to GPU, overwriting previous upload if any.
    void mjr_uploadMesh(const mjModel* m, const mjrContext* con, int meshid);

    # Upload height field to GPU, overwriting previous upload if any.
    void mjr_uploadHField(const mjModel* m, const mjrContext* con, int hfieldid);

    # Make con->currentBuffer current again.
    void mjr_restoreBuffer(const mjrContext* con);

    # Set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN.
    # If only one buffer is available, set that buffer and ignore framebuffer argument.
    void mjr_setBuffer(int framebuffer, mjrContext* con);

    # Read pixels from current OpenGL framebuffer to client buffer.
    # Viewport is in OpenGL framebuffer; client buffer starts at (0,0).
    void mjr_readPixels(unsigned char* rgb, float* depth,
                              mjrRect viewport, const mjrContext* con);

    # Draw pixels from client buffer to current OpenGL framebuffer.
    # Viewport is in OpenGL framebuffer; client buffer starts at (0,0).
    void mjr_drawPixels(const unsigned char* rgb, const float* depth,
                              mjrRect viewport, const mjrContext* con);

    # Blit from src viewpoint in current framebuffer to dst viewport in other framebuffer.
    # If src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR.
    void mjr_blitBuffer(mjrRect src, mjrRect dst, int flg_color, int flg_depth, const mjrContext* con);

    # Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done).
    void mjr_setAux(int index, const mjrContext* con);

    # Blit from Aux buffer to con->currentBuffer.
    void mjr_blitAux(int index, mjrRect src, int left, int bottom, const mjrContext* con);

    # Draw text at (x,y) in relative coordinates; font is mjtFont.
    void mjr_text(int font, const char* txt, const mjrContext* con,
                        float x, float y, float r, float g, float b);

    # Draw text overlay; font is mjtFont; gridpos is mjtGridPos.
    void mjr_overlay(int font, int gridpos, mjrRect viewport,
                           const char* overlay, const char* overlay2, const mjrContext* con);

    # Get maximum viewport for active buffer.
    mjrRect mjr_maxViewport(const mjrContext* con);

    # Draw rectangle.
    void mjr_rectangle(mjrRect viewport, float r, float g, float b, float a);

    # Draw rectangle with centered text.
    void mjr_label(mjrRect viewport, int font, const char* txt,
                   float r, float g, float b, float a, float rt, float gt, float bt,
				           const mjrContext* con);

    # Draw 2D figure.
    void mjr_figure(mjrRect viewport, const mjvFigure* fig, const mjrContext* con);

    # Render 3D scene.
    void mjr_render(mjrRect viewport, mjvScene* scn, const mjrContext* con);

    # Call glFinish.
    void mjr_finish();

    # Call glGetError and return result.
    int mjr_getError();

    # Find first rectangle containing mouse, -1: not found.
    int mjr_findRect(int x, int y, int nrect, const mjrRect* rect);

    #---------------------- UI framework ---------------------------------------------------

    # Add definitions to UI.
    void mjui_add(mjUI* ui, const mjuiDef* _def);

    # Add definitions to UI section.
    void mjui_addToSection(mjUI* ui, int sect, const mjuiDef* _def);


    # Compute UI sizes.
    void mjui_resize(mjUI* ui, const mjrContext* con);

    # Update specific section/item; -1: update all.
    void mjui_update(int section, int item, const mjUI* ui, const mjuiState* state, const mjrContext* con);

    # Handle UI event, return pointer to changed item, NULL if no change.
    mjuiItem* mjui_event(mjUI* ui, mjuiState* state, const mjrContext* con);

    # Copy UI image to current buffer.
    void mjui_render(mjUI* ui, const mjuiState* state, const mjrContext* con);


    #--------------------- Error and memory -----------------------------------------------

    # Main error function; does not return to caller.
    void mju_error(const char* msg);

    # Error function with int argument; msg is a printf format string.
    void mju_error_i(const char* msg, int i);

    # Error function with string argument.
    void mju_error_s(const char* msg, const char* text);

    # Main warning function; returns to caller.
    void mju_warning(const char* msg);

    # Warning function with int argument.
    void mju_warning_i(const char* msg, int i);

    # Warning function with string argument.
    void mju_warning_s(const char* msg, const char* text);

    # Clear user error and memory handlers.
    void mju_clearHandlers();

    # Allocate memory; byte-align on 8; pad size to multiple of 8.
    void* mju_malloc(size_t size);

    # Free memory, using free() by default.
    void mju_free(void* ptr);

    # High-level warning function: count warnings in mjData, print only the first.
    void mj_warning(mjData* d, int warning, int info);

    # Write [datetime, type: message] to MUJOCO_LOG.TXT.
    void mju_writeLog(const char* type, const char* msg);


    #--------------------- Standard math --------------------------------------------------

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


    #----------------------------- Vector math --------------------------------------------

    # Set res = 0.
    void mju_zero3(mjtNum res[3]);

    # Set res = vec.
    void mju_copy3(mjtNum res[3], const mjtNum data[3]);

    # Set res = vec*scl.
    void mju_scl3(mjtNum res[3], const mjtNum vec[3], mjtNum scl);

    # Set res = vec1 + vec2.
    void mju_add3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3]);

    # Set res = vec1 - vec2.
    void mju_sub3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3]);

    # Set res = res + vec.
    void mju_addTo3(mjtNum res[3], const mjtNum vec[3]);

    # Set res = res - vec.
    void mju_subFrom3(mjtNum res[3], const mjtNum vec[3]);

    # Set res = res + vec*scl.
    void mju_addToScl3(mjtNum res[3], const mjtNum vec[3], mjtNum scl);

    # Set res = vec1 + vec2*scl.
    void mju_addScl3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3], mjtNum scl);

    # Normalize vector, return length before normalization.
    mjtNum mju_normalize3(mjtNum res[3]);

    # Return vector length (without normalizing the vector).
    mjtNum mju_norm3(const mjtNum vec[3]);

    # Return dot-product of vec1 and vec2.
    mjtNum mju_dot3(const mjtNum vec1[3], const mjtNum vec2[3]);

    # Return Cartesian distance between 3D vectors pos1 and pos2.
    mjtNum mju_dist3(const mjtNum pos1[3], const mjtNum pos2[3]);

    # Multiply vector by 3D rotation matrix: res = mat * vec.
    void mju_rotVecMat(mjtNum res[3], const mjtNum vec[3], const mjtNum mat[9]);

    # Multiply vector by transposed 3D rotation matrix: res = mat' * vec.
    void mju_rotVecMatT(mjtNum res[3], const mjtNum vec[3], const mjtNum mat[9]);

    # Compute cross-product: res = cross(a, b).
    void mju_cross(mjtNum res[3], const mjtNum a[3], const mjtNum b[3]);

    # Set res = 0.
    void mju_zero4(mjtNum res[4]);

    # Set res = (1,0,0,0).
    void mju_unit4(mjtNum res[4]);

    # Set res = vec.
    void mju_copy4(mjtNum res[4], const mjtNum data[4]);

    # Normalize vector, return length before normalization.
    mjtNum mju_normalize4(mjtNum res[4]);

    # Set res = 0.
    void mju_zero(mjtNum* res, int n);

    # Set res = vec.
    void mju_copy(mjtNum* res, const mjtNum* data, int n);

    # Return sum(vec).
    mjtNum mju_sum(const mjtNum* vec, int n);

    # Return L1 norm: sum(abs(vec)).
    mjtNum mju_L1(const mjtNum* vec, int n);

    # Set res = vec*scl.
    void mju_scl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n);

    # Set res = vec1 + vec2.
    void mju_add(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n);

    # Set res = vec1 - vec2.
    void mju_sub(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n);

    # Set res = res + vec.
    void mju_addTo(mjtNum* res, const mjtNum* vec, int n);

    # Set res = res - vec.
    void mju_subFrom(mjtNum* res, const mjtNum* vec, int n);

    # Set res = res + vec*scl.
    void mju_addToScl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n);

    # Set res = vec1 + vec2*scl.
    void mju_addScl(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, mjtNum scl, int n);

    # Normalize vector, return length before normalization.
    mjtNum mju_normalize(mjtNum* res, int n);

    # Return vector length (without normalizing vector).
    mjtNum mju_norm(const mjtNum* res, int n);

    # Return dot-product of vec1 and vec2.
    mjtNum mju_dot(const mjtNum* vec1, const mjtNum* vec2, const int n);

    # Multiply matrix and vector: res = mat * vec.
    void mju_mulMatVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                             int nr, int nc);

    # Multiply transposed matrix and vector: res = mat' * vec.
    void mju_mulMatTVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                              int nr, int nc);

    # Transpose matrix: res = mat'.
    void mju_transpose(mjtNum* res, const mjtNum* mat, int nr, int nc);

    # Multiply matrices: res = mat1 * mat2.
    void mju_mulMatMat(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                             int r1, int c1, int c2);

    # Multiply matrices, second argument transposed: res = mat1 * mat2'.
    void mju_mulMatMatT(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                              int r1, int c1, int r2);

    # Multiply matrices, first argument transposed: res = mat1' * mat2.
    void mju_mulMatTMat(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                              int r1, int c1, int c2);

    # Set res = mat' * diag * mat if diag is not NULL, and res = mat' * mat otherwise.
    void mju_sqrMatTD(mjtNum* res, const mjtNum* mat, const mjtNum* diag, int nr, int nc);

    # Coordinate transform of 6D motion or force vector in rotation:translation format.
    # rotnew2old is 3-by-3, NULL means no rotation; flg_force specifies force or motion type.
    void mju_transformSpatial(mjtNum res[6], const mjtNum vec[6], int flg_force,
                                    const mjtNum newpos[3], const mjtNum oldpos[3],
                                    const mjtNum rotnew2old[9]);


    #--------------------- Sparse math ----------------------------------------------------

    # Return dot-product of vec1 and vec2, where vec1 is sparse.
    mjtNum mju_dotSparse(const mjtNum* vec1, const mjtNum* vec2,
                               const int nnz1, const int* ind1);

    # Return dot-product of vec1 and vec2, where both vectors are sparse.
    mjtNum mju_dotSparse2(const mjtNum* vec1, const mjtNum* vec2,
                                const int nnz1, const int* ind1,
                                const int nnz2, const int* ind2);

    # Convert matrix from dense to sparse format.
    void mju_dense2sparse(mjtNum* res, const mjtNum* mat, int nr, int nc,
                                int* rownnz, int* rowadr, int* colind);

    # Convert matrix from sparse to dense format.
    void mju_sparse2dense(mjtNum* res, const mjtNum* mat, int nr, int nc,
                                const int* rownnz, const int* rowadr, const int* colind);

    # Multiply sparse matrix and dense vector:  res = mat * vec.
    void mju_mulMatVecSparse(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int nr,
                                   const int* rownnz, const int* rowadr, const int* colind);

    # Compress layout of sparse matrix.
    void mju_compressSparse(mjtNum* mat, int nr, int nc,
                                  int* rownnz, int* rowadr, int* colind);

    # Set dst = a*dst + b*src, return nnz of result, modify dst sparsity pattern as needed.
    # Both vectors are sparse. The required scratch space is 2*n.
    int mju_combineSparse(mjtNum* dst, const mjtNum* src, int n, mjtNum a, mjtNum b,
                                int dst_nnz, int src_nnz, int* dst_ind, const int* src_ind,
                                mjtNum* scratch, int nscratch);

    # Set res = matT * diag * mat if diag is not NULL, and res = matT * mat otherwise.
    # The required scratch space is 3*nc. The result has uncompressed layout.
    void mju_sqrMatTDSparse(mjtNum* res, const mjtNum* mat, const mjtNum* matT,
                                  const mjtNum* diag, int nr, int nc,
                                  int* res_rownnz, int* res_rowadr, int* res_colind,
                                  const int* rownnz, const int* rowadr, const int* colind,
                                  const int* rownnzT, const int* rowadrT, const int* colindT,
                                  mjtNum* scratch, int nscratch);

    # Transpose sparse matrix.
    void mju_transposeSparse(mjtNum* res, const mjtNum* mat, int nr, int nc,
                                  int* res_rownnz, int* res_rowadr, int* res_colind,
                                  const int* rownnz, const int* rowadr, const int* colind);


    #--------------------- Quaternions ----------------------------------------------------

    # Rotate vector by quaternion.
    void mju_rotVecQuat(mjtNum res[3], const mjtNum vec[3], const mjtNum quat[4]);

    # Negate quaternion.
    void mju_negQuat(mjtNum res[4], const mjtNum quat[4]);

    # Muiltiply quaternions.
    void mju_mulQuat(mjtNum res[4], const mjtNum quat1[4], const mjtNum quat2[4]);

    # Muiltiply quaternion and axis.
    void mju_mulQuatAxis(mjtNum res[4], const mjtNum quat[4], const mjtNum axis[3]);

    # Convert axisAngle to quaternion.
    void mju_axisAngle2Quat(mjtNum res[4], const mjtNum axis[3], mjtNum angle);

    # Convert quaternion (corresponding to orientation difference) to 3D velocity.
    void mju_quat2Vel(mjtNum res[3], const mjtNum quat[4], mjtNum dt);

    # Subtract quaternions, express as 3D velocity: qb*quat(res) = qa.
    void mju_subQuat(mjtNum res[3], const mjtNum qa[4], const mjtNum qb[4]);

    # Convert quaternion to 3D rotation matrix.
    void mju_quat2Mat(mjtNum res[9], const mjtNum quat[4]);

    # Convert 3D rotation matrix to quaterion.
    void mju_mat2Quat(mjtNum quat[4], const mjtNum mat[9]);

    # Compute time-derivative of quaternion, given 3D rotational velocity.
    void mju_derivQuat(mjtNum res[4], const mjtNum quat[4], const mjtNum vel[3]);

    # Integrate quaterion given 3D angular velocity.
    void mju_quatIntegrate(mjtNum quat[4], const mjtNum vel[3], mjtNum scale);

    # Construct quaternion performing rotation from z-axis to given vector.
    void mju_quatZ2Vec(mjtNum quat[4], const mjtNum vec[3]);


    #--------------------- Poses ----------------------------------------------------------

    # Multiply two poses.
    void mju_mulPose(mjtNum posres[3], mjtNum quatres[4],
                           const mjtNum pos1[3], const mjtNum quat1[4],
                           const mjtNum pos2[3], const mjtNum quat2[4]);

    # Negate pose.
    void mju_negPose(mjtNum posres[3], mjtNum quatres[4],
                           const mjtNum pos[3], const mjtNum quat[4]);

    # Transform vector by pose.
    void mju_trnVecPose(mjtNum res[3], const mjtNum pos[3], const mjtNum quat[4],
                              const mjtNum vec[3]);


    #--------------------- Decompositions --------------------------------------------------

    # Cholesky decomposition: mat = L*L'; return rank.
    int mju_cholFactor(mjtNum* mat, int n, mjtNum mindiag);

    # Solve mat * res = vec, where mat is Cholesky-factorized
    void mju_cholSolve(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int n);

    # Cholesky rank-one update: L*L' +/- x*x'; return rank.
    int mju_cholUpdate(mjtNum* mat, mjtNum* x, int n, int flg_plus);

    # Eigenvalue decomposition of symmetric 3x3 matrix.
    int mju_eig3(mjtNum* eigval, mjtNum* eigvec, mjtNum* quat, const mjtNum* mat);


    #--------------------- Miscellaneous --------------------------------------------------

    # Muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax).
    mjtNum mju_muscleGain(mjtNum len, mjtNum vel, const mjtNum lengthrange[2],
                          mjtNum acc0, const mjtNum prm[9]);

    # Muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax).
    mjtNum mju_muscleBias(mjtNum len, const mjtNum lengthrange[2],
                          mjtNum acc0, const mjtNum prm[9]);

    # Muscle activation dynamics, prm = (tau_act, tau_deact).
    mjtNum mju_muscleDynamics(mjtNum ctrl, mjtNum act, const mjtNum prm[2]);


    # Convert contact force to pyramid representation.
    void mju_encodePyramid(mjtNum* pyramid, const mjtNum* force,
                                 const mjtNum* mu, int dim);

    # Convert pyramid representation to contact force.
    void mju_decodePyramid(mjtNum* force, const mjtNum* pyramid,
                                 const mjtNum* mu, int dim);

    # Integrate spring-damper analytically, return pos(dt).
    mjtNum mju_springDamper(mjtNum pos0, mjtNum vel0, mjtNum Kp, mjtNum Kv, mjtNum dt);

    # Return min(a,b) with single evaluation of a and b.
    mjtNum mju_min(mjtNum a, mjtNum b);

    # Return max(a,b) with single evaluation of a and b.
    mjtNum mju_max(mjtNum a, mjtNum b);

    # Return sign of x: +1, -1 or 0.
    mjtNum mju_sign(mjtNum x);

    # Round x to nearest integer.
    int mju_round(mjtNum x);

    # Convert type id (mjtObj) to type name.
    const char* mju_type2Str(int type);

    # Convert type name to type id (mjtObj).
    int mju_str2Type(const char* str);

    # Construct a warning message given the warning type and info.
    const char* mju_warningText(int warning, int info);

    # Return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise. Used by check functions.
    int mju_isBad(mjtNum x);

    # Return 1 if all elements are 0.
    int mju_isZero(mjtNum* vec, int n);

    # Standard normal random number generator (optional second number).
    mjtNum mju_standardNormal(mjtNum* num2);

    # Convert from float to mjtNum.
    void mju_f2n(mjtNum* res, const float* vec, int n);

    # Convert from mjtNum to float.
    void mju_n2f(float* res, const mjtNum* vec, int n);

    # Convert from double to mjtNum.
    void mju_d2n(mjtNum* res, const double* vec, int n);

    # Convert from mjtNum to double.
    void mju_n2d(double* res, const mjtNum* vec, int n);

    # Insertion sort, resulting list is in increasing order.
    void mju_insertionSort(mjtNum* list, int n);

    # Integer insertion sort, resulting list is in increasing order.
    void mju_insertionSortInt(int* list, int n);

    # Generate Halton sequence.
    mjtNum mju_Halton(int index, int base);

    # Sigmoid function over 0<=x<=1 constructed from half-quadratics.
    mjtNum mju_sigmoid(mjtNum x);

