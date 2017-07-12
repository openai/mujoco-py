import logging
import multiprocessing as mp
import sys
from time import perf_counter

import numpy as np
from PIL import Image

from mujoco_py import MjSim, load_model_from_path
from mujoco_py.mjrenderpool import RenderPool

logging.basicConfig(stream=sys.stdout, level=logging.INFO)


def main(batch_size=4, img_width=100, n_frames=1, n_workers=1):
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)

    logger.info("starting")

    sim = MjSim(load_model_from_path("xmls/fetch/main.xml"))

    states = []
    for i in range(batch_size):
        sim.data.set_joint_qpos('shoulder_lift_joint', i / batch_size)
        sim.forward()
        states.append(sim.get_state())

    p = RenderPool(
        sim.model, device_ids=8, max_batch_size=batch_size, n_workers=n_workers)

    rgbs = p.render(img_width, img_width, states=states).copy()
    assert rgbs.shape == (batch_size, img_width, img_width, 3)

    times = []
    for _ in range(n_frames):
        start_t = perf_counter()
        p.render(img_width, img_width, states=states)
        times.append(perf_counter() - start_t)

    times = np.array(times)
    t = np.sum(times)
    total_frames = n_frames * batch_size
    logger.info(
        "Completed %d frames in %.3fs: %.3f mspf, %.1f FPS",
        total_frames, t, t / total_frames * 1000, total_frames / t)

    times *= 1000
    logger.info("PER BATCH (n=%d): avg=%.3fms p50=%.3fms p99=%.3fms",
                batch_size, np.mean(times), *np.percentile(times, [50, 99]))

    p.close()
    logger.info("done")

    # n_cells = int(np.ceil(np.sqrt(batch_size)))
    # big_image = np.ones((n_cells * img_width, n_cells * img_width, 3),
    #                     dtype=np.uint8)
    # big_image *= 255
    # for i, img in enumerate(rgbs):
    #     y = (i // n_cells) * img_width
    #     x = (i % n_cells) * img_width
    #     big_image[y:y+img_width, x:x+img_width, :] = img

    # Image.fromarray(big_image).save('out.png')


if __name__ == '__main__':
    mp.set_start_method('spawn')
    batch_size = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    n_workers = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    main(n_frames=1000, batch_size=batch_size, n_workers=n_workers)
