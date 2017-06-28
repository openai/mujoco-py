#!/usr/bin/env python3
"""
Example of how bodies interact with each other. For a body to be able to
move it needs to have joints. In this example, the "robot" is a red ball
with X and Y slide joints (and a Z slide joint that isn't controlled).
On the floor, there's a cylinder with X and Y slide joints, so it can
be pushed around with the robot. There's also a box without joints. Since
the box doesn't have joints, it's fixed and can't be pushed around.
"""
from mujoco_py import load_model_from_xml, MjSim, MjViewer
import math
import os

MODEL_XML = """
<?xml version="1.0" ?>
<mujoco>
    <option timestep="0.005" />
    <worldbody>
        <body name="robot" pos="0 0 1.2">
            <joint axis="1 0 0" damping="0.1" name="slide0" pos="0 0 0" type="slide"/>
            <joint axis="0 1 0" damping="0.1" name="slide1" pos="0 0 0" type="slide"/>
            <joint axis="0 0 1" damping="1" name="slide2" pos="0 0 0" type="slide"/>
            <geom mass="1.0" pos="0 0 0" rgba="1 0 0 1" size="0.15" type="sphere"/>
			<camera euler="0 0 0" fovy="40" name="rgb" pos="0 0 2.5"></camera>
        </body>
        <body mocap="true" name="mocap" pos="0.5 0.5 0.5">
			<geom conaffinity="0" contype="0" pos="0 0 0" rgba="1.0 1.0 1.0 0.5" size="0.1 0.1 0.1" type="box"></geom>
			<geom conaffinity="0" contype="0" pos="0 0 0" rgba="1.0 1.0 1.0 0.5" size="0.2 0.2 0.05" type="box"></geom>
		</body>
        <body name="cylinder" pos="0.1 0.1 0.2">
            <geom mass="1" size="0.15 0.15" type="cylinder"/>
            <joint axis="1 0 0" name="cylinder:slidex" type="slide"/>
            <joint axis="0 1 0" name="cylinder:slidey" type="slide"/>
        </body>
        <body name="box" pos="-0.8 0 0.2">
            <geom mass="0.1" size="0.15 0.15 0.15" type="box"/>
        </body>
        <body name="floor" pos="0 0 0.025">
            <geom condim="3" size="1.0 1.0 0.02" rgba="0 1 0 1" type="box"/>
        </body>
    </worldbody>
    <actuator>
        <motor gear="2000.0" joint="slide0"/>
        <motor gear="2000.0" joint="slide1"/>
    </actuator>
</mujoco>
"""

model = load_model_from_xml(MODEL_XML)
sim = MjSim(model)
viewer = MjViewer(sim)
t = 0
while True:
    sim.data.ctrl[0] = math.cos(t / 10.) * 0.01
    sim.data.ctrl[1] = math.sin(t / 10.) * 0.01
    t += 1
    sim.step()
    viewer.render()
    if t > 100 and os.getenv('TESTING') is not None:
        break
