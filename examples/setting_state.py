#!/usr/bin/env python3
"""
Example for how to modifying the MuJoCo qpos during execution.
"""

import os
from mujoco_py import load_model_from_xml, MjSim, MjViewer

MODEL_XML = """
<?xml version="1.0" ?>
<mujoco>
    <worldbody>
        <body name="box" pos="0 0 0.2">
            <geom size="0.15 0.15 0.15" type="box"/>
            <joint axis="1 0 0" name="box:x" type="slide"/>
            <joint axis="0 1 0" name="box:y" type="slide"/>
        </body>
        <body name="floor" pos="0 0 0.025">
            <geom size="1.0 1.0 0.02" rgba="0 1 0 1" type="box"/>
        </body>
    </worldbody>
</mujoco>
"""


def print_box_xpos(sim):
    print("box xpos:", sim.data.get_body_xpos("box"))


model = load_model_from_xml(MODEL_XML)
sim = MjSim(model)
viewer = MjViewer(sim)

states = [{'box:x': +0.8, 'box:y': +0.8},
          {'box:x': -0.8, 'box:y': +0.8},
          {'box:x': -0.8, 'box:y': -0.8},
          {'box:x': +0.8, 'box:y': -0.8},
          {'box:x': +0.0, 'box:y': +0.0}]

# MjModel.joint_name2id returns the index of a joint in
# MjData.qpos.
x_joint_i = sim.model.get_joint_qpos_addr("box:x")
y_joint_i = sim.model.get_joint_qpos_addr("box:y")

print_box_xpos(sim)

while True:
    for state in states:
        sim_state = sim.get_state()
        sim_state.qpos[x_joint_i] = state["box:x"]
        sim_state.qpos[y_joint_i] = state["box:y"]
        sim.set_state(sim_state)
        sim.forward()
        print("updated state to", state)
        print_box_xpos(sim)
        viewer.render()

    if os.getenv('TESTING') is not None:
        break
