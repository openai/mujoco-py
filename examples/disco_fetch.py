#!/usr/bin/env python3
"""
Displays robot fetch at a disco party.
"""
from mujoco_py import load_model_from_path, MjSim, MjViewer
from mujoco_py.modder import TextureModder
import os

model = load_model_from_path("xmls/fetch/main.xml")
sim = MjSim(model)

viewer = MjViewer(sim)
modder = TextureModder(sim)

t = 0

while True:
    for name in sim.model.geom_names:
        modder.rand_all(name)

    viewer.render()
    t += 1
    if t > 100 and os.getenv('TESTING') is not None:
        break
