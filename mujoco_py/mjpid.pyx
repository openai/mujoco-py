from libc.math cimport fabs, fmax, fmin
from mujoco_py.generated import const

cdef int CONTROLLER_TYPE_PI_CASCADE = 1,

cdef struct PIDErrors:
    double error
    double integral_error
    double derivative_error

cdef struct PIDOutput:
    PIDErrors errors
    double output

cdef struct PIDParameters:
    double dt_seconds  # PID sampling time.
    double setpoint
    double feedback
    double Kp
    double error_deadband
    double integral_max_clamp
    double integral_time_const
    double derivative_gain_smoothing
    double derivative_time_const
    PIDErrors previous_errors

cdef mjtNum c_zero_gains(const mjModel*m, const mjData*d, int id) with gil:
    return 0.0

cdef PIDOutput _pid(PIDParameters parameters):
    """
    A general purpose PID controller implemented in the standard form. 
    Kp == Kp
    Ki == Kp/Ti
    Kd == Kp*Td
    
    In this situation, Kp is a knob to tune the agressiveness, wheras Ti and Td will
    change the response time of the system in a predictable way. Lower Ti or Td means
    that the system will respond to error more quickly/agressively.
    
    error deadband: if set will shrink error within to 0.0
    clamp on integral term:  helps on saturation problem in I.
    derivative smoothing term:  reduces high frequency noise in D.
    
    :param parameters: PID parameters
    :return: A PID output struct containing the control output and the error state
    """

    cdef double error = parameters.setpoint - parameters.feedback

    # Clamp error that's within the error deadband
    if fabs(error) < parameters.error_deadband:
        error = 0.0

    # Compute derivative error
    cdef double derivative_error = (error - parameters.previous_errors.error) / parameters.dt_seconds

    derivative_error = (1.0 - parameters.derivative_gain_smoothing) * parameters.previous_errors.derivative_error + \
                       parameters.derivative_gain_smoothing * derivative_error

    cdef double derivative_error_term = derivative_error * parameters.derivative_time_const

    # Update and clamp integral error
    integral_error = parameters.previous_errors.integral_error
    integral_error += error * parameters.dt_seconds
    integral_error = fmax(-parameters.integral_max_clamp, fmin(parameters.integral_max_clamp, integral_error))

    cdef double integral_error_term = 0.0
    if parameters.integral_time_const != 0:
        integral_error_term = integral_error / parameters.integral_time_const

    f = parameters.Kp * (error + integral_error_term + derivative_error_term)

    return PIDOutput(output=f, errors=PIDErrors(error=error, derivative_error=derivative_error, integral_error=integral_error))

cdef enum USER_DEFINED_ACTUATOR_PARAMS:
    IDX_PROPORTIONAL_GAIN = 0,
    IDX_INTEGRAL_TIME_CONSTANT = 1,
    IDX_INTEGRAL_MAX_CLAMP = 2,
    IDX_DERIVATIVE_TIME_CONSTANT = 3,
    IDX_DERIVATIVE_GAIN_SMOOTHING = 4,
    IDX_ERROR_DEADBAND = 5,

cdef enum USER_DEFINED_CONTROLLER_DATA_PID:
    IDX_INTEGRAL_ERROR = 0,
    IDX_ERROR = 1,
    IDX_DERIVATIVE_ERROR = 2,
    NUM_USER_DATA_PER_ACT_PID = 3,

cdef mjtNum c_pid_bias(const mjModel*m, const mjData*d, int id):
    """
    To activate PID, set gainprm="Kp Ti Td iClamp errBand iSmooth" in a general type actuator in mujoco xml
    """
    cdef double dt_in_sec = m.opt.timestep
    cdef int NGAIN = int(const.NGAIN)

    result = _pid(parameters=PIDParameters(
        dt_seconds=dt_in_sec,
        setpoint=d.ctrl[id],
        feedback=d.actuator_length[id],
        Kp=m.actuator_gainprm[id * NGAIN + IDX_PROPORTIONAL_GAIN],
        error_deadband=m.actuator_gainprm[id * NGAIN + IDX_ERROR_DEADBAND],
        integral_max_clamp=m.actuator_gainprm[id * NGAIN + IDX_INTEGRAL_MAX_CLAMP],
        integral_time_const=m.actuator_gainprm[id * NGAIN + IDX_INTEGRAL_TIME_CONSTANT],
        derivative_gain_smoothing=m.actuator_gainprm[id * NGAIN + IDX_DERIVATIVE_GAIN_SMOOTHING],
        derivative_time_const=m.actuator_gainprm[id * NGAIN + IDX_DERIVATIVE_TIME_CONSTANT],
        previous_errors=PIDErrors(
            error=d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_ERROR],
            derivative_error=d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_DERIVATIVE_ERROR],
            integral_error=d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_INTEGRAL_ERROR],
        )))

    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_ERROR] = result.errors.error
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_DERIVATIVE_ERROR] = result.errors.derivative_error
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_INTEGRAL_ERROR] = result.errors.integral_error

    f = result.output

    cdef double effort_limit_low = m.actuator_forcerange[id * 2]
    cdef double effort_limit_high = m.actuator_forcerange[id * 2 + 1]

    if effort_limit_low != 0.0 or effort_limit_high != 0.0:
        f = fmax(effort_limit_low, fmin(effort_limit_high, f))
    return f

cdef enum USER_DEFINED_ACTUATOR_PARAMS_CASCADE:
    IDX_CAS_PROPORTIONAL_GAIN = 0,
    IDX_CAS_INTEGRAL_TIME_CONSTANT = 1,
    IDX_CAS_INTEGRAL_MAX_CLAMP = 2,
    IDX_CAS_DERIVATIVE_TIME_CONSTANT = 3,
    IDX_CAS_DERIVATIVE_GAIN_SMOOTHING = 4,
    IDX_CAS_PROPORTIONAL_GAIN_V = 5,
    IDX_CAS_INTEGRAL_TIME_CONSTANT_V = 6,
    IDX_CAS_INTEGRAL_MAX_CLAMP_V = 7,
    IDX_CAS_EMA_SMOOTH = 8,
    IDX_CAS_MAX_VEL = 9,

cdef enum USER_DEFINED_CONTROLLER_DATA_CASCADE:
    IDX_CAS_ERROR = 0,
    IDX_CAS_INTEGRAL_ERROR = 1,
    IDX_CAS_DERIVATIVE_ERROR = 2,
    IDX_CAS_INTEGRAL_ERROR_V = 3,
    IDX_CAS_STORED_EMA_SMOOTH = 4,
    NUM_USER_DATA_PER_ACT_CAS = 5,

cdef NUM_USER_DATA_PER_ACT = max(int(NUM_USER_DATA_PER_ACT_PID), int(NUM_USER_DATA_PER_ACT_CAS))

cdef mjtNum c_pi_cascade_bias(const mjModel*m, const mjData*d, int id):
    """
    A cascaded PID-PI controller implementation that can control position
    and velocity setpoints for a given actuator.
    
    The cascaded controller is implemented as a nested position-velocity controller. A PID loop is 
    wrapped around desired position and a PI loop is wrapped around desired velocity. An exponential 
    moving average filter is applied to the commanded position input (d.ctrl). Additionally the controller
    is gravity compensated via the `qfrc_bias` term.

    The PID-PI gains are set as part of gainprm in the following order: 
    `gainprm="  Kp_pos              -> Position loop proportional gain
                Ti_pos              -> Position loop integral time constant
                Ti_max_clamp_pos    -> Position loop integral error clamp
                Td_pos              -> Position loop derivative time constant
                Td_smooth_pos       -> Position loop derivative smoothing (EMA)
                Kp_vel              -> Velocity loop proportional gain
                Ti_vel              -> Velocity loop integral time constant
                max_clamp_vel       -> Velocity loop integral error clamp
                ema_smooth_factor   -> Exponential moving average (EMA) on desired position
                max_vel"`           -> Clamped velocity limit (applied in positive and negative direction)  
    """
    cdef double dt_in_sec = m.opt.timestep
    cdef int NGAIN = int(const.NGAIN)
    cdef double smooth_pos_setpoint;

    cdef double Kp_cas = m.actuator_gainprm[id * NGAIN + IDX_CAS_PROPORTIONAL_GAIN]

    # Apply Exponential Moving Average smoothing to the position setpoint, applying warmstart
    # on the first iteration.
    if d.time > 0.0:
        ctrl_ema = d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_STORED_EMA_SMOOTH]
        smoothing_factor = m.actuator_gainprm[id * NGAIN + IDX_CAS_EMA_SMOOTH]
        smooth_pos_setpoint = (smoothing_factor * ctrl_ema) + (1 - smoothing_factor) * d.ctrl[id]
    else:
        smooth_pos_setpoint = d.ctrl[id]
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_STORED_EMA_SMOOTH] = smooth_pos_setpoint

    if Kp_cas != 0:
        # Run a position PID loop and use the result to set a desired velocity signal
        pos_output = _pid(parameters=PIDParameters(
            dt_seconds=dt_in_sec,
            setpoint=smooth_pos_setpoint,
            feedback=d.actuator_length[id],
            Kp=Kp_cas,
            error_deadband=0.0,
            integral_max_clamp=m.actuator_gainprm[id * NGAIN + IDX_CAS_INTEGRAL_MAX_CLAMP],
            integral_time_const=m.actuator_gainprm[id * NGAIN + IDX_CAS_INTEGRAL_TIME_CONSTANT],
            derivative_gain_smoothing=m.actuator_gainprm[id * NGAIN + IDX_CAS_DERIVATIVE_GAIN_SMOOTHING],
            derivative_time_const=m.actuator_gainprm[id * NGAIN + IDX_CAS_DERIVATIVE_TIME_CONSTANT],
            previous_errors=PIDErrors(
                error=d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_ERROR],
                derivative_error=d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_DERIVATIVE_ERROR],
                integral_error=d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_INTEGRAL_ERROR],
            )))

        # Save errors
        d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_ERROR] = pos_output.errors.error
        d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_DERIVATIVE_ERROR] = pos_output.errors.derivative_error
        d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_INTEGRAL_ERROR] = pos_output.errors.integral_error
        des_vel = pos_output.output
    else:
        # If P gain on position loop is zero, only use the velocity controller
        des_vel = d.ctrl[id]

    # Clamp max angular velocity
    max_qvel = m.actuator_gainprm[id * NGAIN + IDX_CAS_MAX_VEL]
    des_vel = fmax(-max_qvel, fmin(max_qvel, des_vel))

    vel_output = _pid(parameters=PIDParameters(
        dt_seconds=dt_in_sec,
        setpoint=des_vel,
        feedback=d.actuator_velocity[id],
        Kp=m.actuator_gainprm[id * NGAIN + IDX_CAS_PROPORTIONAL_GAIN_V],
        error_deadband=0.0,
        integral_max_clamp=m.actuator_gainprm[id * NGAIN + IDX_CAS_INTEGRAL_MAX_CLAMP_V],
        integral_time_const=m.actuator_gainprm[id * NGAIN + IDX_CAS_INTEGRAL_TIME_CONSTANT_V],
        derivative_gain_smoothing=0.0,
        derivative_time_const=0.0,
        previous_errors=PIDErrors(
            error=0.0,
            derivative_error=0.0,
            integral_error=d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_INTEGRAL_ERROR_V],
        )))

    # Save errors
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_INTEGRAL_ERROR_V] = vel_output.errors.integral_error

    # Limit max torque at the output
    cdef double effort_limit_low = m.actuator_forcerange[id * 2]
    cdef double effort_limit_high = m.actuator_forcerange[id * 2 + 1]

    f = vel_output.output

    # Gravity compensation
    f += d.qfrc_bias[id]

    if effort_limit_low != 0.0 or effort_limit_high != 0.0:
        f = fmax(effort_limit_low, fmin(effort_limit_high, f))

    return f

cdef enum USER_DEFINED_ACTUATOR_DATA:
    IDX_CONTROLLER_TYPE = 0
    NUM_ACTUATOR_DATA = 1

cdef mjtNum c_custom_bias(const mjModel*m, const mjData*d, int id) with gil:
    """
    Switches between PID and Cascaded PID-PI type custom bias computation based on the
    defined actuator's actuator_user field.
    user="1": Cascade PID-PI
    default: PID
    :param m: mjModel
    :param d:  mjData
    :param id: actuator ID
    :return: Custom actuator force
    """
    controller_type = int(m.actuator_user[id * m.nuser_actuator + IDX_CONTROLLER_TYPE])

    if controller_type == CONTROLLER_TYPE_PI_CASCADE:
        return c_pi_cascade_bias(m, d, id)
    return c_pid_bias(m, d, id)

def set_pid_control(m, d):
    global mjcb_act_gain
    global mjcb_act_bias

    if m.nuserdata < m.nu * NUM_USER_DATA_PER_ACT:
        raise Exception('nuserdata is not set large enough to store PID internal states.')

    if m.nuser_actuator < m.nu * NUM_ACTUATOR_DATA:
        raise Exception('nuser_actuator is not set large enough to store controller types')

    for i in range(m.nuserdata):
        d.userdata[i] = 0.0

    mjcb_act_gain = c_zero_gains
    mjcb_act_bias = c_custom_bias
