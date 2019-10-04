from libc.math cimport fabs, fmax, fmin
from mujoco_py.generated import const

"""
  Kp == Kp
  Ki == Kp/Ti
  Kd == Kp/Td

  In this situation, Kp is a knob to tune the agressiveness, wheras Ti and Td will
  change the response time of the system in a predictable way. Lower Ti or Td means
  that the system will respond to error more quickly/agressively.

  error deadband: ERROR_DEADBAND if set will shrink error within to 0.0
  clamp on integral term: INTEGRAL_MAX_CLAMP helps on saturation problem in I.
  derivative smoothing term: DERIVATIVE_GAIN_SMOOTHING reduces high frequency noise in D.

  set in user="kp Ti Td iClamp errBand iSmooth" in mujoco xml.
"""
cdef enum USER_DEFINED_ACTUATOR_PARAMS:
    PROPORTIONAL_GAIN = 0,
    INTEGRAL_TIME_CONSTANT = 1,
    DERIVATIVE_TIME_CONSTANT = 2,
    INTEGRAL_MAX_CLAMP = 3,

    ERROR_DEADBAND = 4,
    DERIVATIVE_GAIN_SMOOTHING = 5

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

    cdef double Kp = m.actuator_user[id * m.nuser_actuator + PROPORTIONAL_GAIN]
    cdef double error_deadband = m.actuator_user[id * m.nuser_actuator + ERROR_DEADBAND]
    cdef double integral_max_clamp = m.actuator_user[id * m.nuser_actuator + INTEGRAL_MAX_CLAMP]
    cdef double integral_time_const = m.actuator_user[id * m.nuser_actuator + INTEGRAL_TIME_CONSTANT]
    cdef double derivative_gain_smoothing = \
        m.actuator_user[id * m.nuser_actuator + DERIVATIVE_GAIN_SMOOTHING]
    cdef double derivate_time_const = m.actuator_user[id * m.nuser_actuator + DERIVATIVE_TIME_CONSTANT]

    cdef double corrective_effort_limit = fmax(
            fabs(m.actuator_forcerange[id * 2]), fabs(m.actuator_forcerange[id * 2 + 1]))

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

    cdef double derivative_error_term = 0.0
    if derivate_time_const != 0:
        derivative_error_term = derivative_error / derivate_time_const

    f = Kp * (error + integral_error_term + derivative_error_term)
    # print(id, error, integral_error_term, derivative_error_term, derivative_error, dt_in_sec,
    #    last_error, integral_error, derivative_error_last)

    d.userdata[id * NUM_USER_DATA_PER_ACT + LAST_ERROR] = error
    d.userdata[id * NUM_USER_DATA_PER_ACT + DERIVATIVE_ERROR_LAST] = derivative_error
    d.userdata[id * NUM_USER_DATA_PER_ACT + INTEGRAL_ERROR] = integral_error
    return fmax(-corrective_effort_limit, fmin(corrective_effort_limit, f))


def set_pid_control(m, d):
    global mjcb_act_gain
    global mjcb_act_bias

    if m.nuserdata < m.nu * NUM_USER_DATA_PER_ACT:
        raise Exception('nuserdata is not set large enough to store PID internal states')

    for i in range(m.nuserdata):
        d.userdata[i] = 0.0

    for i in range(m.nu):
        m.actuator_gaintype[i] = const.GAIN_USER
        m.actuator_biastype[i] = const.BIAS_USER

        # if user does not set, will chose default settings.
        # for kp, it tries to use m.actuator_gainprm[i][0]
        if m.actuator_user[i][PROPORTIONAL_GAIN] <= 0.0:
            m.actuator_user[i][PROPORTIONAL_GAIN] = m.actuator_gainprm[i][0]

        if m.actuator_user[i][INTEGRAL_TIME_CONSTANT] <= 0.0:
            m.actuator_user[i][INTEGRAL_TIME_CONSTANT] = 20

        if m.actuator_user[i][INTEGRAL_MAX_CLAMP] <= 0.0:
            m.actuator_user[i][INTEGRAL_MAX_CLAMP] = 10

        m.actuator_user[i][ERROR_DEADBAND] = 0.0

        if m.actuator_user[i][DERIVATIVE_GAIN_SMOOTHING] <= 0.0:
            m.actuator_user[i][DERIVATIVE_GAIN_SMOOTHING] = 0.1

        if m.actuator_user[i][DERIVATIVE_TIME_CONSTANT] <= 0.0:
            m.actuator_user[i][DERIVATIVE_TIME_CONSTANT] = 25

    mjcb_act_gain = c_zero_gains
    mjcb_act_bias = c_pid_bias
