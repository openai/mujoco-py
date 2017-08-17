import multiprocessing as mp
import os
import subprocess
import sys
from os.path import abspath, dirname, join

import numpy as np
import pytest

from mujoco_py import MjSim, MjRenderPool, load_model_from_xml
from mujoco_py.modder import TextureModder
from mujoco_py.tests.utils import compare_imgs

BASIC_MODEL_XML = """
<mujoco>
    <worldbody>
        <light name="light1" diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <camera name="camera1" pos="3 0 0" zaxis="1 0 0" />
        <geom name="g1" pos="0.5 0.4 0.3" type="plane" size="1 1 0.1" rgba="1 1 1 1" material="m1" />
        <body pos="0 0 1" name="body1">
            <joint name="joint1" type="free"/>
            <geom name="g2" pos="0 1 0" type="box" size=".1 .2 .3" rgba="1 1 1 1" material="m2" />
            <site name="site1" pos="1 0 0" size="0.1" type="sphere"/>
            <site name="sensorsurf" pos="0 0.045 0" size=".03 .03 .03" type="ellipsoid" rgba="0.3 0.2 0.1 0.3"/>
        </body>
        <body pos="1 0 0" name="mocap1" mocap="true">
            <geom name="g3" conaffinity="0" contype="0" pos="0 0 0" size="0.01 0.01 0.01" type="box" material="m3" rgba="1 1 1 1"/>
        </body>
    </worldbody>
    <sensor>
        <touch name="touchsensor" site="sensorsurf" />
    </sensor>
    <asset>
        <texture name="t1" width="33" height="36" type="2d" builtin="flat" />
        <texture name="t2" width="34" height="39" type="2d" builtin="flat" />
        <texture name="t3" width="31" height="37" type="2d" builtin="flat" />
        <material name="m1" texture="t1" />
        <material name="m2" texture="t2" />
        <material name="m3" texture="t3" />
    </asset>
</mujoco>
"""


@pytest.mark.requires_rendering
def test_spawn():
    # pytest and nose both use 'fork' by default, so we
    # expect a runtime error
    model = load_model_from_xml(BASIC_MODEL_XML)
    with pytest.raises(RuntimeError):
        MjRenderPool(model, n_workers=3)


@pytest.mark.requires_rendering
def test_multiprocessing():
    # pytest doesn't work well with multiprocessing, so just
    # run the multiprocessing tests manually by running this
    # script as a subprocess
    env = os.environ
    env['MUJOCO_PY_TEST_ASSET_DIR_PATH'] = abspath(
        join(dirname(__file__), '..', 'test_imgs'))
    subprocess.check_call([sys.executable, __file__], env=env, shell=True)


def mp_test_create_destroy():
    model = load_model_from_xml(BASIC_MODEL_XML)
    pool = MjRenderPool(model, n_workers=2)
    del pool

    # closing before deleting should be fine too
    pool = MjRenderPool(model, n_workers=2)
    pool.close()
    del pool


def mp_test_rendering():
    model = load_model_from_xml(BASIC_MODEL_XML)
    pool = MjRenderPool(model, n_workers=3)

    images = pool.render(100, 100)
    assert images.shape == (3, 100, 100, 3)
    compare_imgs(images[0], 'test_render_pool.mp_test_rendering.0.png')
    assert np.all(images[0] == images[1])

    images, depth = pool.render(101, 103, depth=True)
    assert images.shape == (3, 103, 101, 3)
    assert depth.shape == (3, 103, 101)
    assert np.all(images[0] == images[1])
    assert np.all(images[1] == images[2])


def mp_test_cameras():
    model = load_model_from_xml(BASIC_MODEL_XML)
    pool = MjRenderPool(model, n_workers=1)

    image = pool.render(100, 100)
    assert image.shape == (1, 100, 100, 3)
    compare_imgs(image[0], 'test_render_pool.mp_test_cameras.0.png')

    image = pool.render(100, 100, camera_name='camera1')
    assert image.shape == (1, 100, 100, 3)
    compare_imgs(image[0], 'test_render_pool.mp_test_cameras.1.png')


def mp_test_modder():
    model = load_model_from_xml(BASIC_MODEL_XML)
    pool = MjRenderPool(model, n_workers=2, modder=TextureModder)

    images = pool.render(100, 100, randomize=True)
    assert images.shape == (2, 100, 100, 3)

    # the order of the images aren't guaranteed to be consistent
    # between the render runs
    images1 = pool.render(100, 100, randomize=False)
    assert images1.shape == (2, 100, 100, 3)

    if np.all(images[0] == images1[0]) and np.all(images[1] == images1[1]):
        images_same = True
    elif np.all(images[0] == images1[1]) and np.all(images[1] == images1[0]):
        images_same = True
    else:
        images_same = False
    assert images_same

    images2 = pool.render(100, 100, randomize=True)
    assert images2.shape == (2, 100, 100, 3)

    if np.all(images[0] == images2[0]) and np.all(images[1] == images2[1]):
        images_same = True
    elif np.all(images[0] == images2[1]) and np.all(images[1] == images2[0]):
        images_same = True
    else:
        images_same = False
    assert not images_same


def mp_test_states():
    sim = MjSim(load_model_from_xml(BASIC_MODEL_XML))

    states = []
    for val in range(3):
        sim.data.qpos[:3] = val * 0.1
        states.append(sim.get_state())

    pool = MjRenderPool(sim.model, n_workers=3)

    images = pool.render(100, 100, states=states[:2])
    assert images.shape == (2, 100, 100, 3)
    compare_imgs(images[0], 'test_render_pool.mp_test_states.1.png')
    compare_imgs(images[1], 'test_render_pool.mp_test_states.2.png')

    states = list(reversed(states))
    images = pool.render(100, 100, states=states)
    assert images.shape == (3, 100, 100, 3)
    compare_imgs(images[0], 'test_render_pool.mp_test_states.3.png')
    compare_imgs(images[1], 'test_render_pool.mp_test_states.4.png')
    compare_imgs(images[2], 'test_render_pool.mp_test_states.5.png')


if __name__ == '__main__':
    mp.freeze_support()
    mp.set_start_method('spawn')

    mp_test_create_destroy()
    mp_test_rendering()
    mp_test_states()
    mp_test_cameras()
    mp_test_modder()
