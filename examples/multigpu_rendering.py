"""
This is an example for rendering on multiple GPUs in parallel,
using the multiprocessing module.
"""
from multiprocessing import set_start_method
from time import perf_counter

from mujoco_py import load_model_from_path, MjRenderPool


def main():
    # Image size for rendering
    IMAGE_WIDTH = 255
    IMAGE_HEIGHT = 255
    # Number of frames to render per sim
    N_FRAMES = 100
    # Number of sims to run in parallel (assumes one per GPU),
    # so N_SIMS=2 assumes there are 2 GPUs available.
    N_SIMS = 2

    pool = MjRenderPool(load_model_from_path("xmls/tosser.xml"), device_ids=N_SIMS)

    print("main(): start benchmarking", flush=True)
    start_t = perf_counter()

    for _ in range(N_FRAMES):
        pool.render(IMAGE_WIDTH, IMAGE_HEIGHT)

    t = perf_counter() - start_t
    print("Completed in %.1fs: %.3fms, %.1f FPS" % (
            t, t / (N_FRAMES * N_SIMS) * 1000, (N_FRAMES * N_SIMS) / t),
          flush=True)

    print("main(): finished", flush=True)


if __name__ == "__main__":
    set_start_method('spawn')
    main()
