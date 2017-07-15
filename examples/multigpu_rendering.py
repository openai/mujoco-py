"""
This is an example for rendering on multiple GPUs in parallel,
using the multiprocessing module.
"""
from multiprocessing import set_start_method
from time import perf_counter
from mujoco_py import load_model_from_path, MjSim
from mujoco_py.mjrenderpool import RenderProcess
import tensorflow as tf

#
# Experiment parameters
#

# Image size for rendering
IMAGE_WIDTH = 255
IMAGE_HEIGHT = 255
# Number of frames to render per sim
N_FRAMES = 2000
# Number of sims to run in parallel (assumes one per GPU),
# so N_SIMS=2 assumes there are 2 GPUs available.
N_SIMS = 2


def setup_sim(device_id):
    model = load_model_from_path("xmls/fetch/main.xml")
    sim = MjSim(model)

    image = sim.render(
        IMAGE_WIDTH, IMAGE_HEIGHT, device_id=device_id)
    assert image.shape == (IMAGE_HEIGHT, IMAGE_WIDTH, 3)

    return sim


def update_sim(sim, device_id):
    sim.step()
    return sim.render(IMAGE_WIDTH, IMAGE_HEIGHT, device_id=device_id)


def main():
    print("main(): create processes", flush=True)
    processes = []
    for device_id in range(N_SIMS):
        p = RenderProcess(
            device_id, setup_sim, update_sim,
            (IMAGE_HEIGHT, IMAGE_WIDTH, 3))
        processes.append(p)

    for p in processes:
        p.wait()

    print("main(): start benchmarking", flush=True)
    start_t = perf_counter()

    for _ in range(N_FRAMES):
        for p in processes:
            p.update()

        for p in processes:
            p.wait()

        for p in processes:
            p.read()

    t = perf_counter() - start_t
    print("Completed in %.1fs: %.3fms, %.1f FPS" % (
            t, t / (N_FRAMES * N_SIMS) * 1000, (N_FRAMES * N_SIMS) / t),
          flush=True)

    print("main(): stopping processes", flush=True)
    for p in processes:
        p.stop()

    print("main(): finished", flush=True)


print("XXX about to call main()", flush=True)
if __name__ == "__main__":
    set_start_method('fork')
    main()
