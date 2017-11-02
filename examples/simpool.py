#!/usr/bin/env python
# Runs several simulations in parallel.

from mujoco_py import load_model_from_xml, MjSim, MjSimPool

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
    <actuator>
        <motor gear="2000.0" joint="box:x"/>
        <motor gear="2000.0" joint="box:y"/>
    </actuator>
</mujoco>
"""

model = load_model_from_xml(MODEL_XML)
pool = MjSimPool([MjSim(model) for _ in range(20)])
for i, sim in enumerate(pool.sims):
    sim.data.qpos[:] = 0.0
    sim.data.qvel[:] = 0.0
    sim.data.ctrl[:] = i

# Advance all 20 simulations 100 times.
for _ in range(100):
    pool.step()

for i, sim in enumerate(pool.sims):
    print("%d-th sim qpos=%s" % (i, str(sim.data.qpos)))
