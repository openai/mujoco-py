#!/usr/bin/env python3
"""
Shows how to access internal functions from MuJoCo.
"""
from mujoco_py import load_model_from_xml, MjSim, functions
import numpy as np

MODEL_XML = """
<?xml version="1.0" ?>
<mujoco>
    <option timestep="0.005" />
    <worldbody>
        <body name="robot" pos="0 0 1.2">
            <joint axis="1 0 0" damping="0.1" name="robot:slide0" pos="0 0 0" type="slide"/>
            <geom mass="1.0" pos="0 0 0" rgba="1 0 0 1" size="0.15" type="sphere"/>
        </body>
    </worldbody>
</mujoco>
"""

model = load_model_from_xml(MODEL_XML)
sim = MjSim(model)

print("Nicely exposed function:\n")
print(sim.model.get_xml())

print("\nversus MuJoCo internals:\n\n")

functions.mj_saveLastXML("/tmp/saved.xml", model, "", 0)
with open("/tmp/saved.xml", "r") as f:
    print(f.read())

sim.render(100, 100)

modelpos = np.zeros(3)
modelquat = np.zeros(4)
roompos = np.ones(3)
roomquat = np.array([1., 0., 1., 0.])

functions.mjv_room2model(modelpos, modelquat, roompos,
                         roomquat, sim.render_contexts[0].scn)

print("\n\nAnother internal function, mjv_room2model:")
print("modelpos = %s, modelquat = %s" % (str(modelpos), str(modelquat)))


res = np.zeros(9)
functions.mju_quat2Mat(res, roomquat)
print("\n\nAnother internal function, mju_quat2Mat:\n%s" % res)
