#!/usr/bin/env python3
"""
Shows how to use render callback.
"""
from mujoco_py import load_model_from_path, MjSim, MjViewer
from mujoco_py.modder import TextureModder
import os

modder = None
def render_callback(sim, viewer):
    global modder
    if modder is None:
        modder = TextureModder(sim)
    for name in sim.model.geom_names:
        modder.rand_all(name)

model = load_model_from_path("xmls/fetch/main.xml")
sim = MjSim(model, render_callback=render_callback)

viewer = MjViewer(sim)

t = 0

while True:
    viewer.render()
    t += 1
    if t > 100 and os.getenv('TESTING') is not None:
        break