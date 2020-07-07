from libc.math cimport fabs, fmax, fmin
from mujoco_py.generated import const
import numpy as np

"""
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
    NUM_USER_DATA_PER_ACT = 3,

cdef int CONTROLLER_TYPE_INVERSE_DYNAMICS = 1,

cdef mjtNum c_zero_gains(const mjModel*m, const mjData*d, int id) with gil:
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
   Inverse Dynamics (ID) Controller
   
   qacc:               Joint acceleration.
   qfrc_applied:       Torques applied directly to the joints.
   xfrc_applied:       Cartesian forces applied directly to bodies.
   qfrc_actuator:      Torques applied directly to the actuators.
   Jx'*xfrc_applied:   Joint torque resulting from cartesian forces (xfrc_applied).
   
   qfrc_inverse gives the joint torques necessary to achieve a desired joint acceleration (qacc) given 
   internal and external forces and torques. ID control solves the following torque balance by calling
   mjinverse(model, data):
   
       qfrc_inverse = qfrc_applied + Jx'*xfrc_applied + qfrc_actuator
   The error in desired joint acceleration is wrapped using a PD controller.
   To provide a smooth reference signal for the ID controller, an Exponential Moving Average (EMA) is
   used on the reference control signal (ctrl_ema). 
   """
cdef enum USER_DEFINED_ID_ACTUATOR_PARAMS:
    IDX_ID_PROPORTIONAL_GAIN = 0,
    IDX_ID_DERIVATIVE_GAIN = 1,
    IDX_ID_EMA_SMOOTH = 2,

cdef enum USER_DEFINED_ID_CONTROLLER_DATA:
    IDX_CTRL_REF = 0

cdef mjtNum c_id_bias(const mjModel*m, const mjData*d, int id):
    cdef int NGAIN = int(const.NGAIN)
    ctrl_ema = d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CTRL_REF]
    ema_smooth = m.actuator_gainprm[id * NGAIN + IDX_ID_EMA_SMOOTH]
    cdef double effort_limit_low = m.actuator_forcerange[id * 2]
    cdef double effort_limit_high = m.actuator_forcerange[id * 2 + 1]

    # Apply an Exponential Moving Average (EMA) to desired control
    ctrl_ema = (ema_smooth * ctrl_ema) + (1 - ema_smooth) * d.ctrl[id]

    qpos_des = ctrl_ema
    qvel_des = 0

    qpos_error = qpos_des - d.qpos[id]
    qvel_error = qvel_des - d.qvel[id]

    # PD gains for desired acceleration
    kp = m.actuator_gainprm[id * NGAIN + IDX_ID_PROPORTIONAL_GAIN]
    kd = m.actuator_gainprm[id * NGAIN + IDX_ID_DERIVATIVE_GAIN]

    # Set desired acceleration of all DoFs (model.nv) to zero except the target actuator [id]
    qacc_des = np.zeros(m.nv)
    qacc_des[id] = kp * qpos_error + kd * qvel_error

    # Set the target forward dyanmics
    for i in range(m.nv):
        d.qacc[i] = qacc_des[i]

    # Compute the inverse dynamics and get the joint torque
    mj_inverse(m, d)
    joint_torque = d.qfrc_inverse[id]

    # Write the joint torque
    f = joint_torque

    # Clip joint torque to be within forcerange if specified
    if effort_limit_low != 0.0 or effort_limit_high != 0.0:
        f = fmax(effort_limit_low, fmin(effort_limit_high, f))

    # Save smooth control signal in userdata
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_CTRL_REF] = ctrl_ema

    return f

cdef enum USER_DEFINED_ACTUATOR_DATA:
    IDX_CONTROLLER_TYPE = 0
    NUM_ACTUATOR_DATA = 1

cdef mjtNum c_custom_bias(const mjModel*m, const mjData*d, int id) with gil:
    """
    Switches between PID and ID type custom bias computation based on the
    defined actuator's actuator_user field.
    user="1": ID
    default: PID
    :param m: mjModel
    :param d:  mjData
    :param id: actuator ID
    :return: Custom actuator force
    """
    controller_type = m.actuator_user[id * m.nuser_actuator + IDX_CONTROLLER_TYPE]

    if controller_type == CONTROLLER_TYPE_INVERSE_DYNAMICS:
        return c_id_bias(m, d, id)
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
