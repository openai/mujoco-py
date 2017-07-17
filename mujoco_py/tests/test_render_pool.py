import multiprocessing as mp
import subprocess
import sys

import pytest

from mujoco_py import MjSim, load_model_from_xml
from mujoco_py.mjrenderpool import MjRenderPool
from mujoco_py.tests.utils import requires_rendering


BASIC_MODEL_XML = """
<mujoco>
    <worldbody>
        <light name="light1" diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <camera name="camera1" pos="3 0 0" zaxis="1 0 0" />
        <geom name="geom1" pos="0.5 0.4 0.3" type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
        <body pos="0 0 1" name="body1">
            <joint name="joint1" type="free"/>
            <geom name="geom2" pos="0 1 0" type="box" size=".1 .2 .3" rgba="0 .9 0 1"/>
            <site name="site1" pos="1 0 0" size="0.1" type="sphere"/>
            <site name="sensorsurf" pos="0 0.045 0" size=".03 .03 .03" type="ellipsoid" rgba="0.3 0.2 0.1 0.3"/>
        </body>
        <body pos="1 0 0" name="mocap1" mocap="true">
            <geom conaffinity="0" contype="0" pos="0 0 0" size="0.01 0.01 0.01" type="box"/>
        </body>
    </worldbody>
    <sensor>
        <touch name="touchsensor" site="sensorsurf" />
    </sensor>
</mujoco>
"""


def test_spawn():
    # pytest and nose both use 'fork' by default, so we
    # expect a runtime error
    model = load_model_from_xml(BASIC_MODEL_XML)
    with pytest.raises(RuntimeError):
        MjRenderPool(model, n_workers=3)


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

    images, depth = pool.render(101, 103, depth=True)
    assert images.shape == (3, 103, 101, 3)
    assert depth.shape == (3, 103, 101)


def mp_test_states():
    sim = MjSim(load_model_from_xml(BASIC_MODEL_XML))

    states = []
    for val in range(3):
        sim.data.qpos[:3] = val * 0.1
        states.append(sim.get_state())

    pool = MjRenderPool(sim.model, n_workers=3)

    images = pool.render(100, 100, states=states[:2])
    assert images.shape == (2, 100, 100, 3)

    states = list(reversed(states))
    images = pool.render(100, 100, states=states)
    assert images.shape == (3, 100, 100, 3)


@requires_rendering
def test_multiprocessing():
    # pytest doesn't work well with multiprocessing, so just
    # run the multiprocessing tests manually by running this
    # script as a subprocess
    subprocess.check_call([sys.executable, __file__])


if __name__ == '__main__':
    mp.freeze_support()
    mp.set_start_method('spawn')

    mp_test_create_destroy()
    mp_test_rendering()
    mp_test_states()
