import numpy as np
import pytest
import random
from mujoco_py import (MjSim, load_model_from_xml, cymj)

MODEL_XML = """
<mujoco model="inverted pendulum">
	<size nuserdata="100" nuser_actuator="{nuser_actuator}"/>
	<compiler inertiafromgeom="true"/>
	<default>
		<joint armature="0" damping="1" limited="true"/>
		<geom contype="0" friction="1 0.1 0.1" rgba="0.7 0.7 0 1"/>
		<tendon/>
		<motor ctrlrange="-3 3"/>
	</default>
	<option gravity="0 0 -9.81" integrator="RK4" timestep="0.001"/>
	<size nstack="3000"/>
	<worldbody>
        <camera name="camera1" pos="3 0 0" zaxis="1 0 0" />

		<geom name="rail" pos="0 0 0" quat="0.707 0 0.707 0" rgba="0.3 0.3 0.7 1" size="0.02 1" type="capsule"/>
		<body name="cart" pos="0 0 0">
			<geom name="cart" pos="0 0 0" quat="0.707 0 0.707 0" size="0.1 0.1" type="capsule"/>
			<body name="pole" pos="0 0 0">
				<joint axis="0 1 0" name="hinge" pos="0 0 0" range="-90 90" type="hinge"/>
				<geom fromto="0 0 0 0.001 0 0.6" name="cpole" rgba="0 0.7 0.7 1" size="0.049 0.3" type="capsule"/>
			</body>
		</body>
	</worldbody>
	<actuator>
		{actuator}
	</actuator>
</mujoco>
"""

PID_ACTUATOR = """
	<general ctrlrange='-1 1' gaintype="user" biastype="user" forcerange="-100 100" gainprm="200 10 10.0 0.1 0.1 0" joint="hinge" name="a-hinge"/>
"""

CASCADED_PIPI_ACTUATOR = """
	<general ctrlrange='-1 1' gaintype="user" biastype="user" forcerange="-3 3" gainprm="5 0 0 0 0 10 .1 1.5 .97 3" joint="hinge" name="a-hinge" user="1"/>
"""

CASCADED_PDPI_ACTUATOR = """
	<general ctrlrange='-1 1' gaintype="user" biastype="user" forcerange="-3 3" gainprm="10 0 0 .1 1 10 .1 1.5 .97 3" joint="hinge" name="a-hinge" user="1"/>
"""

CASCADED_PDPI_ACTUATOR_NO_D = """
	<general ctrlrange='-1 1' gaintype="user" biastype="user" forcerange="-3 3" gainprm="10 0 0 0 0 10 .1 1.5 .97 3" joint="hinge" name="a-hinge" user="1"/>
"""

P_ONLY_ACTUATOR = """
	<general ctrlrange='-1 1' gaintype="user" biastype="user" gainprm="200" joint="hinge" name="a-hinge"/>
"""

POSITION_ACTUATOR = """
	<position ctrlrange='-1 1' kp=200 joint="hinge" name="a-hinge"/>
"""


def test_mj_pid():
    """
    To enable PID control in the mujoco, please refer to the setting in the PID_ACTUATOR.

    Here we set Kp = 200, Ti = 10, Td = 0.1 (also iClamp = 10.0, dSmooth be 0.1)
    """
    random.seed(30)
    xml = MODEL_XML.format(actuator=PID_ACTUATOR, nuser_actuator=1)
    model = load_model_from_xml(xml)
    sim = MjSim(model)
    cymj.set_pid_control(sim.model, sim.data)

    # Perturbation of pole to be unbalanced
    init_pos = 0.1 * (random.random() - 0.5)
    print('init pos', init_pos)
    sim.data.qpos[0] = init_pos

    pos = 0.0
    sim.data.ctrl[0] = pos
    print('desired position:', pos)

    for _ in range(1000):
        sim.step()

    print('final pos', sim.data.qpos[0])
    assert abs(sim.data.qpos[0] - pos) < 1e-4


def test_mj_proportional_only():
    """
    Check new PID control is backward compatible with position control
    when it only has a Kp term.
    """
    model = load_model_from_xml(MODEL_XML.format(actuator=P_ONLY_ACTUATOR, nuser_actuator=1))
    sim = MjSim(model)
    cymj.set_pid_control(sim.model, sim.data)

    model2 = load_model_from_xml(MODEL_XML.format(actuator=POSITION_ACTUATOR, nuser_actuator=1))
    sim2 = MjSim(model2)

    init_pos = 0.1 * (random.random() - 0.5)
    sim.data.qpos[0] = sim2.data.qpos[0] = init_pos
    sim.data.ctrl[0] = sim2.data.ctrl[0] = 0

    for i in range(2000):
        print(i, sim.data.qpos[0], sim2.data.qpos[0])
        sim.step()
        sim2.step()
        assert abs(sim.data.qpos[0] - sim2.data.qpos[0]) <= 1e-7, "%d step violates" % i


def test_cascaded_pipi():
    """
    To enable Cascaded control in mujoco, please refer to the setting in 
    the CASCADED_PIPI_ACTUATOR. user param should be set to 1

    Here we set Kp = 5 for the position control loop and Kp =  10 for the velocity control
    Ti = 0.1 and integral_max_clamp=1.5.
    EMA smoothing constant is set to 0.97, and velocity limit is 3 rad/s
    """
    random.seed(30)
    xml = MODEL_XML.format(actuator=CASCADED_PIPI_ACTUATOR, nuser_actuator=1)
    model = load_model_from_xml(xml)
    sim = MjSim(model)
    cymj.set_pid_control(sim.model, sim.data)

    # Perturbation of pole to be unbalanced
    init_pos = 0.1 * (random.random() - 1.0)
    print('init pos', init_pos)
    sim.data.qpos[0] = init_pos

    desired_pos = 0.0
    sim.data.ctrl[0] = desired_pos
    print('desired position:', desired_pos)

    max_torque = 0

    for _ in range(1000):
        sim.step()
        if abs(sim.data.actuator_force[0]) > max_torque:
            max_torque = abs(sim.data.actuator_force[0])

    print('final pos', sim.data.qpos[0])
    assert abs(sim.data.qpos[0] - desired_pos) < 1e-3
    assert max_torque <= 3  # Torque limit set on the actuator


def test_cascaded_pdpi():
    """
    This tests using a PD-PI loop with the cascaded controller. This is achieved by setting the 
    integral time constant and clamp equal to zero.

    Here we setup two sims (CASCADED_PDPI_ACTUATOR and CASCADED_PDPI_ACTUATOR_NO_D), one with 
    derivative gain and one without. With the exception of the derivative term, all other gain 
    parameters are the same. The goal is to show the stability added by the derivative term. 
    """
    random.seed(30)
    xml = MODEL_XML.format(actuator=CASCADED_PDPI_ACTUATOR, nuser_actuator=1)
    model = load_model_from_xml(xml)
    sim = MjSim(model)
    cymj.set_pid_control(sim.model, sim.data)

    """
    sim2 uses the same Kp gain on the position loop as sim but does not have any derivative gain. 
    It is expected that given this Kp gain and no derivative gain, the system will be unstable 
    and fail to hold the desired position. 
    """
    xml = MODEL_XML.format(actuator=CASCADED_PDPI_ACTUATOR_NO_D, nuser_actuator=1)
    model2 = load_model_from_xml(xml)
    sim2 = MjSim(model2)
    cymj.set_pid_control(sim2.model, sim2.data)

    # Perturbation of pole to be unbalanced
    init_pos = 0.1 * (random.random() - 1.0)
    print('init pos', init_pos)
    sim.data.qpos[0] = sim2.data.qpos[0] = init_pos

    desired_pos = 0.0
    sim.data.ctrl[0] = sim2.data.ctrl[0] = desired_pos
    print('desired position:', desired_pos)

    max_torque = 0

    for _ in range(2000):
        sim.step()
        sim2.step()
        if abs(sim.data.actuator_force[0]) > max_torque:
            max_torque = abs(sim.data.actuator_force[0])

    print('final pos', sim.data.qpos[0])
    assert abs(sim2.data.qpos[0] - desired_pos) > 1e-3  # Verify instability without D term
    assert abs(sim.data.qpos[0] - desired_pos) < 1e-3  # Verify stability with D term
    assert max_torque <= 3  # Torque limit set on the actuator


def test_mjsize_exception():
    """nuser_actuator must be set large enough to use custom controllers."""
    xml = MODEL_XML.format(actuator=CASCADED_PIPI_ACTUATOR, nuser_actuator=0)
    model = load_model_from_xml(xml)
    sim = MjSim(model)
    with pytest.raises(Exception):
        cymj.set_pid_control(sim.model, sim.data)


CASCADED_PDPI_ACTUATOR_NO_EMA = """
	<general ctrlrange='-1 1' gaintype="user" biastype="user" forcerange="-3 3" gainprm="10 0 0 .1 1 10 .1 1.5 0 3" joint="hinge" name="a-hinge" user="1"/>
"""


def test_warm_start_ema():
    """
    Test that the smoothed commanded position is initialized to the commanded position at
    start.
    """
    # Load model with EMA smoothing (0.97)
    model = load_model_from_xml(MODEL_XML.format(actuator=CASCADED_PDPI_ACTUATOR, nuser_actuator=1))
    sim = MjSim(model)
    cymj.set_pid_control(sim.model, sim.data)
    # Load model without EMA smoothing (0.0)
    model2 = load_model_from_xml(MODEL_XML.format(actuator=CASCADED_PDPI_ACTUATOR_NO_EMA, nuser_actuator=1))
    sim2 = MjSim(model2)
    cymj.set_pid_control(sim2.model, sim2.data)
    init_pos = 0.1 * (random.random() - 0.5)
    sim.data.ctrl[0] = sim2.data.ctrl[0] = .2
    sim.step()
    sim2.step()
    EMA_SMOOTH_IDX = 4
    # Compare that the saved smoothed position is the same for both EMA=.97 and EMA=0
    assert abs(sim.data.userdata[EMA_SMOOTH_IDX] - sim2.data.userdata[EMA_SMOOTH_IDX]) < 1e-10
