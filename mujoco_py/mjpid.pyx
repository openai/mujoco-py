from libc.math cimport fabs, fmax, fmin
from mujoco_py.generated import const

"""
  Kp == Kp
  Ki == Kp/Ti
  Kd == Kp*Td

  In this situation, Kp is a knob to tune the agressiveness, wheras Ti and Td will
  change the response time of the system in a predictable way. Lower Ti or Td means
  that the system will respond to error more quickly/agressively.

  error deadband: ERROR_DEADBAND if set will shrink error within to 0.0
  clamp on integral term: INTEGRAL_MAX_CLAMP helps on saturation problem in I.
  derivative smoothing term: DERIVATIVE_GAIN_SMOOTHING reduces high frequency noise in D.

  set in user="Ti Td iClamp errBand iSmooth" in mujoco xml.
"""
cdef enum USER_DEFINED_ACTUATOR_PARAMS:
    PROPORTIONAL_GAIN = 0,
    INTEGRAL_TIME_CONSTANT = 1,
    INTEGRAL_MAX_CLAMP = 2,
    DERIVATIVE_TIME_CONSTANT = 3,
    DERIVATIVE_GAIN_SMOOTHING = 4,
    ERROR_DEADBAND = 5,


cdef enum USER_DEFINED_CONTROLLER_DATA:
    INTEGRAL_ERROR = 0,
    LAST_ERROR = 1,
    DERIVATIVE_ERROR_LAST = 2,
    NUM_USER_DATA_PER_ACT = 3,


cdef mjtNum c_zero_gains(const mjModel* m, const mjData* d, int id) with gil:
    return 0.0


cdef mjtNum c_pid_bias(const mjModel* m, const mjData* d, int id) with gil:
    cdef double dt_in_sec = m.opt.timestep
    cdef double error = d.ctrl[id] - d.actuator_length[id]
    cdef int NGAIN = int(const.NGAIN)

    cdef double Kp = m.actuator_gainprm[id * NGAIN + PROPORTIONAL_GAIN]
    cdef double error_deadband = m.actuator_gainprm[id * NGAIN + ERROR_DEADBAND]
    cdef double integral_max_clamp = m.actuator_gainprm[id * NGAIN + INTEGRAL_MAX_CLAMP]
    cdef double integral_time_const = m.actuator_gainprm[id * NGAIN + INTEGRAL_TIME_CONSTANT]
    cdef double derivative_gain_smoothing = \
        m.actuator_gainprm[id * NGAIN  + DERIVATIVE_GAIN_SMOOTHING]
    cdef double derivate_time_const = m.actuator_gainprm[id * NGAIN + DERIVATIVE_TIME_CONSTANT]

    cdef double effort_limit_low = m.actuator_forcerange[id * 2]
    cdef double effort_limit_high = m.actuator_forcerange[id * 2 + 1]

    if fabs(error) < error_deadband:
        error = 0.0

    integral_error = d.userdata[id * NUM_USER_DATA_PER_ACT + INTEGRAL_ERROR]
    integral_error += error * dt_in_sec
    integral_error = fmax(-integral_max_clamp, fmin(integral_max_clamp, integral_error))

    last_error = d.userdata[id * NUM_USER_DATA_PER_ACT + LAST_ERROR]
    cdef double derivative_error = (error - last_error) / dt_in_sec

    derivative_error_last = d.userdata[id * NUM_USER_DATA_PER_ACT + DERIVATIVE_ERROR_LAST]

    derivative_error = (1.0 - derivative_gain_smoothing) * derivative_error_last + \
        derivative_gain_smoothing * derivative_error

    cdef double integral_error_term = 0.0
    if integral_time_const != 0:
        integral_error_term = integral_error / integral_time_const

    cdef double derivative_error_term = derivative_error * derivate_time_const

    f = Kp * (error + integral_error_term + derivative_error_term)
    print(id, d.ctrl[id], d.actuator_length[id], error, integral_error_term, derivative_error_term,
        derivative_error, dt_in_sec, last_error, integral_error, derivative_error_last, f)

    d.userdata[id * NUM_USER_DATA_PER_ACT + LAST_ERROR] = error
    d.userdata[id * NUM_USER_DATA_PER_ACT + DERIVATIVE_ERROR_LAST] = derivative_error
    d.userdata[id * NUM_USER_DATA_PER_ACT + INTEGRAL_ERROR] = integral_error
    return fmax(effort_limit_low, fmin(effort_limit_high, f))


def set_pid_control(m, d):
    global mjcb_act_gain
    global mjcb_act_bias

    if m.nuserdata < m.nu * NUM_USER_DATA_PER_ACT:
        raise Exception('nuserdata is not set large enough to store PID internal states')

    for i in range(m.nuserdata):
        d.userdata[i] = 0.0

    mjcb_act_gain = c_zero_gains
    mjcb_act_bias = c_pid_bias
