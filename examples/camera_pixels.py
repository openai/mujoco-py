#!/usr/bin/env python
# demonstration of pixels (drawing buffers on top of viewer)
# in this demo, the pixel buffer is from rendering a moving camera,
# but in general any buffer of pixel values can be used.

import time
import os
import numpy as np
from mujoco_py import load_model_from_xml, MjSim, MjViewer

MODEL_XML = """
<?xml version="1.0" ?>
<mujoco>
    <worldbody>
        <body pos="0 0 0.5">
            <geom size="0.1 0.1 0.1" type="box" rgba="1 0 0 1"/>
            <joint type="slide" axis="1 0 0"/>
            <joint type="slide" axis="0 1 0"/>
        </body>
        <body pos="0 0 1.0">
            <geom size="0.1 0.1 0.1" type="box" rgba="1 0 1 1"/>
            <joint type="slide" axis="1 0 0"/>
            <joint type="slide" axis="0 1 0"/>
        </body>
        <body pos="0 0 1.5">
            <geom size="0.1 0.1 0.1" type="box" rgba="0 0 1 1"/>
            <joint type="slide" axis="1 0 0"/>
            <joint type="slide" axis="0 1 0"/>
        </body>
        <body pos="0 0 2.0">
            <geom size="0.1 0.1 0.1" type="box" rgba="0 1 1 1"/>
            <joint type="slide" axis="1 0 0"/>
            <joint type="slide" axis="0 1 0"/>
        </body>
        <body pos="0 0 5">
            <geom size="0.15 0.15 0.15" type="sphere"/>
            <joint type="slide" axis="1 0 0"/>
            <joint type="slide" axis="0 1 0"/>
            <camera name="cam"/>
        </body>
        <geom size="1 1 1" type="plane" pos="0 0 0" name="floor"/>
    </worldbody>
</mujoco>
"""

model = load_model_from_xml(MODEL_XML)
sim = MjSim(model)
viewer = MjViewer(sim)
step = 0


while True:
    # Move the boxes and the camera
    t = time.time()
    x, y = np.cos(t), np.sin(t)
    sim.data.qpos[:] = [x, y, -x, y, x, -y, -x, -y, x * .3, y * .3]
    sim.forward()

    # draw an image drawn from the camera to the screen
    render = sim.render(200, 200, mode="offscreen", camera_name="cam")
    viewer.add_pixels(render, 300, 100)

    viewer.render()

    step += 1
    if step > 100 and os.getenv('TESTING') is not None:
        break
