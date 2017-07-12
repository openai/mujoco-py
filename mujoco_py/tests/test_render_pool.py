import logging
import multiprocessing as mp

from mujoco_py.mjrenderpool import RenderPool

info = mp.get_logger().info


def main():
    logger = mp.log_to_stderr()
    logger.setLevel(logging.INFO)

    info("main: starting")

    p = RenderPool(None, device_ids=2, n_workers=3)
    rgbs = p.render(2, 2)
    print("rgbs")
    print(rgbs)
    # p.render(3, 3)

    p.close()
    info("main: done")

    import ipdb
    ipdb.set_trace()


if __name__ == '__main__':
    mp.set_start_method('spawn')
    main()
