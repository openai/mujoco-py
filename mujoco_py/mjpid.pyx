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
    IDX_PROPORTIONAL_GAIN_V = 3,
    IDX_INTEGRAL_TIME_CONSTANT_V = 4,
    IDX_INTEGRAL_MAX_CLAMP_V = 5,
    IDX_EMA_SMOOTH_V = 6,


cdef enum USER_DEFINED_CONTROLLER_DATA:
    IDX_INTEGRAL_ERROR = 0,
    IDX_INTEGRAL_ERROR_V = 1,
    IDX_LAST_ERROR = 2,
    IDX_LAST_ERROR_V = 3,
    IDX_STORED_EMA_SMOOTH_V = 4,
    NUM_USER_DATA_PER_ACT = 5,

cdef int CONTROLLER_TYPE_INVERSE_DYNAMICS = 1,

cdef mjtNum c_zero_gains(const mjModel*m, const mjData*d, int id) with gil:
    return 0.0

cdef mjtNum c_pid_bias(const mjModel*m, const mjData*d, int id):

    # Get time
    cdef double dt_in_sec = m.opt.timestep
    cdef int NGAIN = int(const.NGAIN)


    ########## Position PI Loop

    # Compute error from d.ctrl[id] setpoint
    cdef double error = d.ctrl[id] - d.actuator_length[id]

    # Proportional term
    cdef double Kp = m.actuator_gainprm[id * NGAIN + IDX_PROPORTIONAL_GAIN]

    # Integral Error
    cdef double integral_max_clamp = m.actuator_gainprm[id * NGAIN + IDX_INTEGRAL_MAX_CLAMP]
    cdef double integral_time_const = m.actuator_gainprm[id * NGAIN + IDX_INTEGRAL_TIME_CONSTANT]
    integral_error = d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_INTEGRAL_ERROR]
    integral_error += error * dt_in_sec

    integral_error = fmax(-integral_max_clamp, fmin(integral_max_clamp, integral_error))
    cdef double integral_error_term = 0.0
    if integral_time_const != 0:
        integral_error_term = integral_error / integral_time_const

    # Save errors
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_LAST_ERROR] = error
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_INTEGRAL_ERROR] = integral_error

    # If P gain on position loop is zero, only use the velocity controller
    if Kp != 0:
        des_vel = Kp * (error + integral_error_term)
    else:
        des_vel = d.ctrl[id]

    # Clamp max angular velocity
    QVEL_MAX = np.pi
    des_vel = fmax(-QVEL_MAX, fmin(QVEL_MAX, des_vel))




    ########## Velocity PI Loop

    ctrl_ema_V = d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_STORED_EMA_SMOOTH_V]
    ema_smooth_V = m.actuator_gainprm[id * NGAIN + IDX_EMA_SMOOTH_V]

    ctrl_ema_V = (ema_smooth_V * ctrl_ema_V) + (1 - ema_smooth_V) * des_vel

    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_STORED_EMA_SMOOTH_V] = ctrl_ema_V

    # Compute error from d.ctrl[id] setpoint
    cdef double error_V = ctrl_ema_V - d.actuator_velocity[id]

    # Proportional term
    cdef double Kp_V = m.actuator_gainprm[id * NGAIN + IDX_PROPORTIONAL_GAIN_V]

    # Integral Error
    cdef double integral_max_clamp_V = m.actuator_gainprm[id * NGAIN + IDX_INTEGRAL_MAX_CLAMP_V]
    cdef double integral_time_const_V = m.actuator_gainprm[id * NGAIN + IDX_INTEGRAL_TIME_CONSTANT_V]
    integral_error_V = d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_INTEGRAL_ERROR_V]
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
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_LAST_ERROR_V] = error_V
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_INTEGRAL_ERROR_V] = integral_error_V


    f += d.qfrc_bias[id]

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
cdef enum USER_DEFINED_INV_DYN_ACTUATOR_PARAMS:
    IDX_INV_DYN_PROPORTIONAL_GAIN = 0,  # Kp, proportional gain
    IDX_INV_DYN_DERIVATIVE_GAIN = 1,  #Kd, derivative gain
    IDX_INV_DYN_EMA_SMOOTH = 2,  #Exponential moving average smoothing factor

cdef enum USER_DEFINED_INV_DYN_CONTROLLER_DATA:
    IDX_INV_DYN_CTRL_REF = 0  # stored value of EMA-smoothed control

cdef mjtNum c_inv_dyn_bias(const mjModel*m, const mjData*d, int id):
    cdef int NGAIN = int(const.NGAIN)
    ctrl_ema = d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_INV_DYN_CTRL_REF]
    ema_smooth = m.actuator_gainprm[id * NGAIN + IDX_INV_DYN_EMA_SMOOTH]
    cdef double effort_limit_low = m.actuator_forcerange[id * 2]
    cdef double effort_limit_high = m.actuator_forcerange[id * 2 + 1]

    # Apply an Exponential Moving Average (EMA) to desired control
    ctrl_ema = (ema_smooth * ctrl_ema) + (1 - ema_smooth) * d.ctrl[id]

    qpos_des = ctrl_ema
    qvel_des = 0

    qpos_error = qpos_des - d.qpos[id]
    qvel_error = qvel_des - d.qvel[id]

    # PD gains for desired acceleration
    kp = m.actuator_gainprm[id * NGAIN + IDX_INV_DYN_PROPORTIONAL_GAIN]
    kd = m.actuator_gainprm[id * NGAIN + IDX_INV_DYN_DERIVATIVE_GAIN]

    # Set desired acceleration of all DoFs (model.nv) to zero except the target actuator [id]
    qacc_des = np.zeros(m.nv)
    qacc_des[id] = kp * qpos_error + kd * qvel_error

    # Set the target forward dyanmics
    for i in range(m.nv):
        d.qacc[i] = qacc_des[i]

    # Compute the inverse dynamics and get the joint torque
    mj_inverse(m, d)
    # print('test')
    # Write the joint torque
    f = d.qfrc_inverse[id]

    # Clip joint torque to be within forcerange if specified
    if effort_limit_low != 0.0 or effort_limit_high != 0.0:
        f = fmax(effort_limit_low, fmin(effort_limit_high, f))

    # Save smooth control signal in userdata
    d.userdata[id * NUM_USER_DATA_PER_ACT + IDX_INV_DYN_CTRL_REF] = ctrl_ema

    return f

cdef enum USER_DEFINED_ACTUATOR_DATA:
    IDX_CONTROLLER_TYPE = 0
    NUM_ACTUATOR_DATA = 1

cdef mjtNum c_custom_bias(const mjModel*m, const mjData*d, int id) with gil:
    """
    Switches between PID and Inverse Dynamics (ID) type custom bias computation based on the
    defined actuator's actuator_user field.
    user="1": ID
    default: PID
    :param m: mjModel
    :param d:  mjData
    :param id: actuator ID
    :return: Custom actuator force
    """
    controller_type = int(m.actuator_user[id * m.nuser_actuator + IDX_CONTROLLER_TYPE])

    if controller_type == CONTROLLER_TYPE_INVERSE_DYNAMICS:
        return c_inv_dyn_bias(m, d, id)
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
