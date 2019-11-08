import numpy as np
import pytest
import random
from mujoco_py import (MjSim, load_model_from_xml, cymj)

MODEL_XML = """
<mujoco model="inverted pendulum">
	<size nuserdata="100"/>
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

P_ONLY_ACTUATOR = """
	<general ctrlrange='-1 1' gaintype="user" biastype="user" gainprm="200" joint="hinge" name="a-hinge"/>
"""

POSITION_ACTUATOR = """
	<position ctrlrange='-1 1' kp=200 joint="hinge" name="a-hinge"/>
"""


"""
	To enable PID control in the mujoco, please
	refer to the setting in the PID_ACTUATOR.

	Here we set Kp = 200, Ti = 10, Td = 0.1 (also iClamp = 10.0, dSmooth be 0.1)
"""
def test_mj_pid():
	xml = MODEL_XML.format(actuator=PID_ACTUATOR)
	model = load_model_from_xml(xml)
	sim = MjSim(model)
	cymj.set_pid_control(sim.model, sim.data)

	# pertubation of pole to be unbalanced
	init_pos = 0.1 * (random.random() - 0.5)
	print('init pos', init_pos)
	sim.data.qpos[0] = init_pos

	pos = 0.0
	sim.data.ctrl[0] = pos
	print('desire position:', pos)

	for _ in range(100):
		sim.step()

	print('final pos', sim.data.qpos[0])
	assert abs(sim.data.qpos[0] - pos) < 0.01

"""
    check new PID control is backward compatible with  position control
	when only has Kp term.
"""
def test_mj_proptional_only():
	model = load_model_from_xml(MODEL_XML.format(actuator=P_ONLY_ACTUATOR))
	sim = MjSim(model)
	cymj.set_pid_control(sim.model, sim.data)

	model2 = load_model_from_xml(MODEL_XML.format(actuator=POSITION_ACTUATOR))
	sim2 = MjSim(model2)

	init_pos = 0.1 * (random.random() - 0.5)
	sim.data.qpos[0] = sim2.data.qpos[0] = init_pos
	sim.data.ctrl[0] = sim2.data.ctrl[0] = 0

	for i in range(2000):
		print(i, sim.data.qpos[0], sim2.data.qpos[0])
		sim.step()
		sim2.step()
		assert abs(sim.data.qpos[0] - sim2.data.qpos[0]) <= 1e-7, "%d step violates" % i
