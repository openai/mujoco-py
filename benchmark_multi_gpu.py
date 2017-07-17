"""
This is an example for rendering on multiple GPUs in parallel,
using the multiprocessing module.
"""
from time import perf_counter
# import numpy as np
from mujoco_py import load_model_from_path, MjSim, MjSimPool
import sys


#
# Experiment parameters
#

# Image size for rendering
IMAGE_WIDTH = 100
IMAGE_HEIGHT = 100
# Number of frames to render per sim
N_FRAMES = 10
# Number of sims to run in parallel (assumes one per GPU),
# so N_SIMS=2 assumes there are 2 GPUs available.
N_SIMS = int(sys.argv[1])


def setup_sim(model, device_id):
    sim = MjSim(model)

    image = sim.render(
        IMAGE_WIDTH, IMAGE_HEIGHT, device_id=device_id)
    assert image.shape == (IMAGE_HEIGHT, IMAGE_WIDTH, 3)

    return sim


def main():
    print("main: start", flush=True)
    model = load_model_from_path("xmls/fetch/main.xml")
    sim_pool = MjSimPool([setup_sim(model, device_id)
                         for device_id in range(N_SIMS)])

    print("main(): start benchmarking", flush=True)
    start_t = perf_counter()

    for _ in range(N_FRAMES):
        sim_pool.render(IMAGE_WIDTH, IMAGE_HEIGHT)

    t = perf_counter() - start_t
    print("Completed in %.1fs: %.3fms, %.1f FPS" % (
            t, t / (N_FRAMES * N_SIMS) * 1000, (N_FRAMES * N_SIMS) / t),
          flush=True)


if __name__ == "__main__":
    main()
