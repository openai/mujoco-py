from libc.math cimport fabs, fmax, fmin
from mujoco_py.generated import const
import numpy as np

"""
  PID Controller Implementation
  Kp == Kp
  Ki == Kp/Ti
  Kd == Kp*Td

  In this situation, Kp is a knob to tune the agressiveness, wheras Ti and Td will
  change the response time of the system in a predictable way. Lower Ti or Td means
  that the system will respond to error more quickly/agressively.

  error deadband: if set will shrink error within to 0.0
  clamp on integral term:  helps on saturation problem in I.
  derivative smoothing term:  reduces high frequency noise in D.

  set in gainprm="Kp Ti Td iClamp errBand iSmooth" in mujoco xml.
"""
cdef enum USER_DEFINED_ACTUATOR_PARAMS:
    IDX_PROPORTIONAL_GAIN = 0,
    IDX_INTEGRAL_TIME_CONSTANT = 1,
    IDX_INTEGRAL_MAX_CLAMP = 2,
    IDX_DERIVATIVE_TIME_CONSTANT = 3,
    IDX_DERIVATIVE_GAIN_SMOOTHING = 4,
    IDX_ERROR_DEADBAND = 5,


cdef enum USER_DEFINED_CONTROLLER_DATA:
    IDX_INTEGRAL_ERROR = 0,
    IDX_LAST_ERROR = 1,
    IDX_DERIVATIVE_ERROR_LAST = 2,
    # Needs to be max() of userdata needed for all control modes. 5 needed for cascasded PI
    NUM_USER_DATA_PER_ACT = 5,


cdef int CONTROLLER_TYPE_PI_CASCADE = 1,

cdef mjtNum c_zero_gains(const mjModel* m, const mjData* d, int id) with gil:
    return 0.0


cdef mjtNum c_pid_bias(const mjModel*m, const mjData*d, int id):
    cdef double dt_in_sec = m.opt.timestep
    cdef double error = d.ctrl[id] - d.actuator_length[id]
    cdef int NGAIN = int(const.NGAIN)

    cdef double Kp = m.actuator_gainprm[id * NGAIN + IDX_PROPORTIONAL_GAIN]
    cdef double error_deadband = m.actuator_gainprm[id * NGAIN + IDX_ERROR_DEADBAND]
    cdef double integral_max_clamp = m.actuator_gainprm[id * NGAIN + IDX_INTEGRAL_MAX_CLAMP]
    cdef double integral_time_const = m.actuator_gainprm[id * NGAIN + IDX_INTEGRAL_TIME_CONSTANT]
    cdef double derivative_gain_smoothing = \
        m.actuator_gainprm[id * NGAIN + IDX_DERIVATIVE_GAIN_SMOOTHING]
    cdef double derivate_time_const = m.actuator_gainprm[id * NGAIN + IDX_DERIVATIVE_TIME_CONSTANT]

    cdef double effort_limit_low = m.actuator_forcerange[id * 2]
    cdef double effort_limit_high = m.actuator_forcerange[id * 2 + 1]

    if fabs(error) < error_deadband:
        error = 0.0

    integral_error = d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_INTEGRAL_ERROR]
    integral_error += error * dt_in_sec
    integral_error = fmax(-integral_max_clamp, fmin(integral_max_clamp, integral_error))

    last_error = d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_LAST_ERROR]
    cdef double derivative_error = (error - last_error) / dt_in_sec

    derivative_error_last = d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_DERIVATIVE_ERROR_LAST]

    derivative_error = (1.0 - derivative_gain_smoothing) * derivative_error_last + \
                       derivative_gain_smoothing * derivative_error

    cdef double integral_error_term = 0.0
    if integral_time_const != 0:
        integral_error_term = integral_error / integral_time_const

    cdef double derivative_error_term = derivative_error * derivate_time_const

    f = Kp * (error + integral_error_term + derivative_error_term)
    # print(id, d.ctrl[id], d.actuator_length[id], error, integral_error_term, derivative_error_term,
    #    derivative_error, dt_in_sec, last_error, integral_error, derivative_error_last, f)

    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_LAST_ERROR] = error
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_DERIVATIVE_ERROR_LAST] = derivative_error
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_INTEGRAL_ERROR] = integral_error

    if effort_limit_low != 0.0 or effort_limit_high != 0.0:
        f = fmax(effort_limit_low, fmin(effort_limit_high, f))
    return f

"""
    Cascaded PI controller

    Two cascaded PI controllers for tracking position and velocity error.
"""


cdef enum USER_DEFINED_ACTUATOR_PARAMS_CASCADE:
    IDX_CAS_PROPORTIONAL_GAIN = 0,
    IDX_CAS_INTEGRAL_TIME_CONSTANT = 1,
    IDX_CAS_INTEGRAL_MAX_CLAMP = 2,
    IDX_CAS_PROPORTIONAL_GAIN_V = 3,
    IDX_CAS_INTEGRAL_TIME_CONSTANT_V = 4,
    IDX_CAS_INTEGRAL_MAX_CLAMP_V = 5,
    IDX_CAS_EMA_SMOOTH_V = 6,


cdef enum USER_DEFINED_CONTROLLER_DATA_CASCADE:
    IDX_CAS_INTEGRAL_ERROR = 0,
    IDX_CAS_INTEGRAL_ERROR_V = 1,
    IDX_CAS_LAST_ERROR = 2,
    IDX_CAS_LAST_ERROR_V = 3,
    IDX_CAS_STORED_EMA_SMOOTH_V = 4,


cdef mjtNum c_pi_cascade_bias(const mjModel*m, const mjData*d, int id):

    # Get time
    cdef double dt_in_sec = m.opt.timestep
    cdef int NGAIN = int(const.NGAIN)


    ########## Position PI Loop

    # Compute error from d.ctrl[id] setpoint
    cdef double error_p_cas = d.ctrl[id] - d.actuator_length[id]

    # Proportional term
    cdef double Kp_cas = m.actuator_gainprm[id * NGAIN + IDX_CAS_PROPORTIONAL_GAIN]

    # Integral Error
    cdef double integral_max_clamp = m.actuator_gainprm[id * NGAIN + IDX_CAS_INTEGRAL_MAX_CLAMP]
    cdef double integral_time_const = m.actuator_gainprm[id * NGAIN + IDX_CAS_INTEGRAL_TIME_CONSTANT]
    integral_error = d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_INTEGRAL_ERROR]
    integral_error += error_p_cas * dt_in_sec

    integral_error = fmax(-integral_max_clamp, fmin(integral_max_clamp, integral_error))
    cdef double integral_error_term = 0.0
    if integral_time_const != 0:
        integral_error_term = integral_error / integral_time_const

    # Save errors
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_LAST_ERROR] = error_p_cas
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_INTEGRAL_ERROR] = integral_error

    # If P gain on position loop is zero, only use the velocity controller
    if Kp_cas != 0:
        des_vel = Kp_cas * (error_p_cas + integral_error_term)
    else:
        des_vel = d.ctrl[id]

    # Clamp max angular velocity
    QVEL_MAX = np.pi
    des_vel = fmax(-QVEL_MAX, fmin(QVEL_MAX, des_vel))




    ########## Velocity PI Loop

    ctrl_ema_V = d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_STORED_EMA_SMOOTH_V]
    ema_smooth_V = m.actuator_gainprm[id * NGAIN + IDX_CAS_EMA_SMOOTH_V]

    ctrl_ema_V = (ema_smooth_V * ctrl_ema_V) + (1 - ema_smooth_V) * des_vel

    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_STORED_EMA_SMOOTH_V] = ctrl_ema_V

    # Compute error from d.ctrl[id] setpoint
    cdef double error_V = ctrl_ema_V - d.actuator_velocity[id]

    # Proportional term
    cdef double Kp_V = m.actuator_gainprm[id * NGAIN + IDX_CAS_PROPORTIONAL_GAIN_V]

    # Integral Error
    cdef double integral_max_clamp_V = m.actuator_gainprm[id * NGAIN + IDX_CAS_INTEGRAL_MAX_CLAMP_V]
    cdef double integral_time_const_V = m.actuator_gainprm[id * NGAIN + IDX_CAS_INTEGRAL_TIME_CONSTANT_V]
    integral_error_V = d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_INTEGRAL_ERROR_V]
    integral_error_V += error_V * dt_in_sec
    integral_error_V = fmax(-integral_max_clamp_V, fmin(integral_max_clamp_V, integral_error_V))

    cdef double integral_error_term_V = 0.0
    if integral_time_const_V != 0:
        integral_error_term_V = integral_error_V / integral_time_const_V

    f = Kp_V * (error_V + integral_error_term_V)

    # Limit max torque
    cdef double effort_limit_low_V = m.actuator_forcerange[id * 2]
    cdef double effort_limit_high_V = m.actuator_forcerange[id * 2 + 1]
    if effort_limit_low_V != 0.0 or effort_limit_high_V != 0.0:
        f = fmax(effort_limit_low_V, fmin(effort_limit_high_V, f))

    # Save errors
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_LAST_ERROR_V] = error_V
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CAS_INTEGRAL_ERROR_V] = integral_error_V


    f += d.qfrc_bias[id]

    return f



cdef enum USER_DEFINED_ACTUATOR_DATA:
    IDX_CONTROLLER_TYPE = 0
    NUM_ACTUATOR_DATA = 1

cdef mjtNum c_custom_bias(const mjModel*m, const mjData*d, int id) with gil:
    """
    Switches between PID and Cascaded PI type custom bias computation based on the
    defined actuator's actuator_user field.
    user="1": Cascade PI
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
