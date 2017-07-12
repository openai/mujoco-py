import logging
import multiprocessing as mp
from time import perf_counter

import numpy as np
from PIL import Image

from mujoco_py import MjSim, load_model_from_path
from mujoco_py.mjrenderpool import RenderPool

logger = mp.get_logger()


def main(n_workers=4, img_width=100, n_frames=1):
    logger = mp.log_to_stderr()
    logger.setLevel(logging.INFO)

    logger.info("main: starting")

    sim = MjSim(load_model_from_path("xmls/fetch/main.xml"))

    states = []
    for i in range(n_workers):
        sim.data.set_joint_qpos('shoulder_lift_joint', i / n_workers)
        sim.forward()
        states.append(sim.get_state())

    p = RenderPool(sim.model, n_workers=n_workers)

    rgbs = p.render(img_width, img_width, states=states).copy()

    start_t = perf_counter()
    for _ in range(n_frames):
        p.render(img_width, img_width, states=states)

    t = perf_counter() - start_t
    total_frames = n_frames * n_workers
    logger.info(
        "Completed %d frames in %.3fs: %.3f mspf, %.1f FPS",
        total_frames, t, t / total_frames * 1000, total_frames / t)

    p.close()
    logger.info("main: done")

    n_cells = int(np.ceil(np.sqrt(n_workers)))
    big_image = np.ones((n_cells * img_width, n_cells * img_width, 3),
                        dtype=np.uint8)
    big_image *= 255
    for i, img in enumerate(rgbs):
        y = (i // n_cells) * img_width
        x = (i % n_cells) * img_width
        big_image[y:y+img_width, x:x+img_width, :] = img

    Image.fromarray(big_image).save('out.png')


if __name__ == '__main__':
    mp.set_start_method('spawn')
    main(n_frames=1000, n_workers=3)
