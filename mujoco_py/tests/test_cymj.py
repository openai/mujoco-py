import pytest
import unittest
from numbers import Number
from io import BytesIO, StringIO
import numpy as np
from numpy.testing import assert_array_equal, assert_array_almost_equal
from mujoco_py import (MjSim, load_model_from_xml,
                       load_model_from_path, MjSimState,
                       ignore_mujoco_warnings,
                       load_model_from_mjb)
from mujoco_py import const, cymj
from mujoco_py.tests.utils import compare_imgs
import scipy.misc
from threading import Thread, Event
from multiprocessing import get_context
import sys


BASIC_MODEL_XML = """
<mujoco>
    <worldbody>
        <light name="light1" diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <camera name="camera1" pos="3 0 0" zaxis="1 0 0" />
        <camera name="camera2" pos="4 0 0" zaxis="1 0 0" />
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

def test_nested():
    model = load_model_from_xml(BASIC_MODEL_XML)
    model.vis.global_.fovy
    model.vis.quality.shadowsize


def test_mj_sim_basics():
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model, nsubsteps=2)

    sim.reset()
    sim.step()
    sim.reset()
    sim.forward()


@pytest.mark.requires_rendering
def test_arrays_of_objs():
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    sim.forward()
    renderer = cymj.MjRenderContext(sim, offscreen=True)
    assert len(renderer.scn.camera) == 2, "Expecting scn.camera to be available"


def test_model_save_load():
    model = load_model_from_xml(BASIC_MODEL_XML)
    xml_from_model = model.get_xml()
    model_from_xml = load_model_from_xml(xml_from_model)
    assert(xml_from_model == model_from_xml.get_xml())
    mjb_from_model = model.get_mjb()
    model_from_mjb = load_model_from_mjb(mjb_from_model)
    assert(mjb_from_model == model_from_mjb.get_mjb())


def test_sim_save():
    model = load_model_from_xml(BASIC_MODEL_XML)
    assert model.nkey == 0
    sim = MjSim(model)

    with StringIO() as f:
        sim.save(f)

        f.seek(0)
        loaded_model = load_model_from_xml(f.read())

        assert loaded_model.nkey == 1

    with BytesIO() as f:
        sim.save(f, format='mjb')

        f.seek(0)
        loaded_model = load_model_from_mjb(f.read())
        assert loaded_model.nkey == 1


def test_mj_sim_buffers():
    model = load_model_from_xml(BASIC_MODEL_XML)

    # test no callback
    sim = MjSim(model, nsubsteps=2)
    assert(sim.udd_state == {})

    sim.step()
    assert(sim.udd_state == {})

    # test with callback
    foo = 10
    d = {"foo": foo,
         "foo_2": np.array([foo, foo])}

    def udd_callback(sim):
        return d

    sim = MjSim(model, nsubsteps=2, udd_callback=udd_callback)
    assert(sim.udd_state is not None)
    assert(sim.udd_state["foo"] == foo)
    assert(sim.udd_state["foo_2"].shape[0] == 2)
    assert(sim.udd_state["foo_2"][0] == foo)

    foo = 11
    d = {"foo": foo,
         "foo_2": np.array([foo, foo])}
    sim.step()
    assert(sim.udd_state is not None)
    assert(sim.udd_state["foo"] == foo)
    assert(sim.udd_state["foo_2"][0] == foo)

    d = {}
    with pytest.raises(AssertionError):
        sim.step()

    d = {"foo": foo,
         "foo_2": np.array([foo, foo]),
         "foo_3": foo}
    with pytest.raises(AssertionError):
        sim.step()

    d = {"foo": foo,
         "foo_2": np.array([foo, foo, foo])}
    with pytest.raises(AssertionError):
        sim.step()

    d = {"foo": "haha",
         "foo_2": np.array([foo, foo, foo])}
    with pytest.raises(AssertionError):
        sim.step()

def test_data_attribute_getters():
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    sim.forward()

    assert_array_equal(sim.data.get_body_xpos("body1"), [0, 0, 1])
    with pytest.raises(ValueError):
        sim.data.get_body_xpos("body_foo")
    with pytest.raises(RuntimeError):
        sim.data.get_xpos("body1")
    assert len(sim.data.get_body_xquat("body1")) == 4
    assert_array_equal(sim.data.get_body_xmat("body1").shape, (3, 3))
    # At (0, 1, 1) since the geom is displaced in the body
    assert_array_equal(sim.data.get_body_xipos("body1"), [0, 1, 1])

    assert_array_equal(sim.data.get_site_xpos("site1"), [1, 0, 1])
    assert_array_equal(sim.data.get_site_xmat("site1").shape, (3, 3))
    assert_array_equal(sim.data.get_geom_xpos("geom1"), [0.5, 0.4, 0.3])
    assert_array_equal(sim.data.get_geom_xpos("geom2"), [0, 1, 1])
    assert_array_equal(sim.data.get_geom_xmat("geom2").shape, (3, 3))
    assert_array_equal(sim.data.get_light_xpos("light1"), [0, 0, 3])
    assert_array_equal(sim.data.get_light_xdir("light1"), [0, 0, -1])
    assert_array_equal(sim.data.get_camera_xpos("camera1"), [3, 0, 0])
    assert_array_equal(sim.data.get_camera_xmat("camera1").shape, (3, 3))

    assert_array_equal(sim.data.get_joint_xaxis("joint1"), [0, 0, 1])
    assert_array_equal(sim.data.get_joint_xanchor("joint1"), [0, 0, 1])


def test_joint_qpos_qvel_ops():
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    sim.forward()

    # Test setting one with a list
    sim.data.set_joint_qpos("joint1", [1, 2, 3, 1, 0, 0, 0])
    # And the other with an np.ndarray
    sim.data.set_joint_qvel("joint1", np.array([1, 2, 3, 0.1, 0.1, 0.1]))
    sim.forward()
    assert_array_equal(sim.data.get_joint_qpos(
        "joint1"), [1, 2, 3, 1, 0, 0, 0])
    assert_array_equal(sim.data.get_joint_qvel(
        "joint1"), [1, 2, 3, 0.1, 0.1, 0.1])


def test_mocap_ops():
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    sim.forward()

    assert_array_equal(sim.data.get_body_xpos("mocap1"), [1, 0, 0])
    assert_array_equal(sim.data.get_mocap_pos("mocap1"), [1, 0, 0])
    assert_array_equal(sim.data.get_mocap_quat("mocap1"), [1, 0, 0, 0])
    new_pos = [2, 1, 1]
    new_quat = [0.707107, 0.707107, 0, 0]
    sim.data.set_mocap_pos("mocap1", new_pos)
    sim.data.set_mocap_quat("mocap1", new_quat)
    sim.forward()
    assert_array_equal(sim.data.get_mocap_pos("mocap1"), new_pos)
    assert_array_almost_equal(sim.data.get_mocap_quat("mocap1"), new_quat)
    assert_array_equal(sim.data.get_body_xpos("mocap1"), new_pos)
    assert_array_almost_equal(sim.data.get_body_xquat("mocap1"), new_quat)
    assert_array_almost_equal(sim.data.get_body_xmat("mocap1"),
                              [[1, 0, 0], [0, 0, -1], [0, 1, 0]])


def test_sim_state():
    model = load_model_from_xml(BASIC_MODEL_XML)

    foo = 10
    d = {"foo": foo,
         "foo_array": np.array([foo, foo, foo]),
         "foo_2darray": np.reshape(np.array([foo, foo, foo, foo]), (2, 2)),
         }

    def udd_callback(sim):
        return d

    sim = MjSim(model, nsubsteps=2, udd_callback=udd_callback)

    state = sim.get_state()
    assert np.array_equal(state.time, sim.data.time)
    assert np.array_equal(state.qpos, sim.data.qpos)
    assert np.array_equal(state.qvel, sim.data.qvel)
    assert np.array_equal(state.act, sim.data.act)
    for k in state.udd_state.keys():
        if (isinstance(state.udd_state[k], Number)):
            assert state.udd_state[k] == sim.udd_state[k]
        else:
            assert np.array_equal(state.udd_state[k], sim.udd_state[k])

    # test flatten, unflatten
    a = state.flatten()

    assert len(a) == (1 + sim.model.nq + sim.model.nv + sim.model.na + 8)

    state2 = MjSimState.from_flattened(a, sim)

    assert np.array_equal(state.time, sim.data.time)
    assert np.array_equal(state.qpos, sim.data.qpos)
    assert np.array_equal(state.qvel, sim.data.qvel)
    assert np.array_equal(state.act, sim.data.act)
    for k in state2.udd_state.keys():
        if (isinstance(state2.udd_state[k], Number)):
            assert state2.udd_state[k] == sim.udd_state[k]
        else:
            assert np.array_equal(state2.udd_state[k], sim.udd_state[k])

    assert state2 == state
    assert not state2 != state

    # test equality with deleting keys
    state2 = state2._replace(udd_state={"foo": foo})
    assert state2 != state
    assert not (state2 == state)

    # test equality with changing contents of array
    state2 = state2._replace(
        udd_state={"foo": foo, "foo_array": np.array([foo, foo + 1])})
    assert state2 != state
    assert not (state2 == state)

    # test equality with adding keys
    d2 = dict(d)
    d2.update({"not_foo": foo})
    state2 = state2._replace(udd_state=d2)
    assert state2 != state
    assert not (state2 == state)

    # test defensive copy
    sim.set_state(state)
    state.qpos[0] = -1
    assert not np.array_equal(state.qpos, sim.data.qpos)

    state3 = sim.get_state()
    state3.qpos[0] = -1
    assert not np.array_equal(state3.qpos, sim.data.qpos)
    state3.udd_state["foo_array"][0] = -1
    assert not np.array_equal(
        state3.udd_state["foo_array"], sim.udd_state["foo_array"])

    # test no callback
    sim = MjSim(model, nsubsteps=2)
    state = sim.get_state()
    print("state.udd_state = %s" % state.udd_state)

    assert state.udd_state == {}

    # test flatten, unflatten
    a = state.flatten()

    assert len(a) == 1 + sim.model.nq + sim.model.nv + sim.model.na

    state2 = MjSimState.from_flattened(a, sim)

    assert np.array_equal(state.time, sim.data.time)
    assert np.array_equal(state.qpos, sim.data.qpos)
    assert np.array_equal(state.qvel, sim.data.qvel)
    assert np.array_equal(state.act, sim.data.act)
    assert state.udd_state == sim.udd_state


def test_mj_warning_raises():
    ''' Test that MuJoCo warnings cause exceptions. '''
    # Two boxes on a plane need more than 1 contact (nconmax)
    xml = '''
    <mujoco>
      <size nconmax="1"/>
      <worldbody>
        <geom type="plane" size="1 1 0.1"/>
        <body pos="1 0 1"> <joint type="free"/> <geom size="1"/> </body>
        <body pos="0 1 1"> <joint type="free"/> <geom size="1"/> </body>
      </worldbody>
    </mujoco>
    '''
    model = load_model_from_xml(xml)
    sim = MjSim(model)

    sim.reset()
    with pytest.raises(Exception):
        # This should raise an exception due to the mujoco warning callback
        sim.step()


def test_ignore_mujoco_warnings():
    # Two boxes on a plane need more than 1 contact (nconmax)
    xml = '''
    <mujoco>
      <size nconmax="1"/>
      <worldbody>
        <geom type="plane" size="1 1 0.1"/>
        <body pos="1 0 1"> <joint type="free"/> <geom size="1"/> </body>
        <body pos="0 1 1"> <joint type="free"/> <geom size="1"/> </body>
      </worldbody>
    </mujoco>
    '''
    model = load_model_from_xml(xml)
    sim = MjSim(model)

    sim.reset()
    with ignore_mujoco_warnings():
        # This should raise an exception due to the mujoco warning callback,
        # but it's suppressed by the context manager.
        sim.step()

    sim.reset()
    with pytest.raises(Exception):
        # test to make sure previous warning callback restored.
        sim.step()


def test_jacobians():
    xml = """
    <mujoco>
        <worldbody>
            <body name="body1" pos="0 0 0">
                <joint axis="1 0 0" name="a" pos="0 0 0" type="hinge"/>
                <geom name="geom1" pos="0 0 0" size="1.0"/>
                <body name="body2" pos="0 0 1">
                    <joint name="b" axis="1 0 0" pos="0 0 1" type="hinge"/>
                    <geom name="geom2" pos="1 1 1" size="0.5"/>
                    <site name="target" size="0.1"/>
                </body>
            </body>
        </worldbody>
        <actuator>
            <motor joint="a"/>
            <motor joint="b"/>
        </actuator>
    </mujoco>
    """
    model = load_model_from_xml(xml)
    sim = MjSim(model)
    sim.reset()
    # After reset jacobians are all zeros
    target_jacp = np.zeros(3 * sim.model.nv)
    sim.data.get_site_jacp('target', jacp=target_jacp)
    np.testing.assert_allclose(target_jacp, np.zeros(3 * sim.model.nv))
    # After first forward, jacobians are real
    sim.forward()
    sim.data.get_site_jacp('target', jacp=target_jacp)
    target_test = np.array([0, 0, -1, 1, 0, 0])
    np.testing.assert_allclose(target_jacp, target_test)
    # Should be unchanged after steps (zero action)
    for _ in range(2):
        sim.step()
        sim.forward()
    sim.data.get_site_jacp('target', jacp=target_jacp)
    assert np.linalg.norm(target_jacp - target_test) < 1e-3
    # Apply a very large action, ensure jacobian unchanged after step
    sim.reset()
    sim.forward()
    sim.data.ctrl[:] = np.ones(sim.model.nu) * 1e9
    sim.step()
    sim.data.get_site_jacp('target', jacp=target_jacp)
    np.testing.assert_allclose(target_jacp, target_test)
    # After large action, ensure jacobian changed after forward
    sim.forward()
    sim.data.get_site_jacp('target', jacp=target_jacp)
    assert not np.allclose(target_jacp, target_test)
    # Test the `site_jacp` property, which gets all at once
    np.testing.assert_allclose(target_jacp, sim.data.site_jacp[0])
    # Test not passing in array
    sim.reset()
    sim.forward()
    target_jacp = sim.data.get_site_jacp('target')
    np.testing.assert_allclose(target_jacp, target_test)
    # Test passing in bad array (long instead of double)
    target_jacp = np.zeros(3 * sim.model.nv, dtype=np.long)
    with pytest.raises(ValueError):
        sim.data.get_site_jacp('target', jacp=target_jacp)
    # Test rotation jacobian - like above but 'jacr' instead of 'jacp'
    # After reset jacobians are all zeros
    sim.reset()
    target_jacr = np.zeros(3 * sim.model.nv)
    sim.data.get_site_jacr('target', jacr=target_jacr)
    np.testing.assert_allclose(target_jacr, np.zeros(3 * sim.model.nv))
    # After first forward, jacobians are real
    sim.forward()
    sim.data.get_site_jacr('target', jacr=target_jacr)
    target_test = np.array([1, 1, 0, 0, 0, 0])
    # Test allocating dedicated array
    target_jacr = sim.data.get_site_jacr('target')
    np.testing.assert_allclose(target_jacr, target_test)
    # Test the batch getter (all sites at once)
    np.testing.assert_allclose(target_jacr, sim.data.site_jacr[0])
    # Test passing in bad array
    target_jacr = np.zeros(3 * sim.model.nv, dtype=np.long)
    with pytest.raises(ValueError):
        sim.data.get_site_jacr('target', jacr=target_jacr)


def test_xvelp():  # xvelp = positional velocity in world frame
    xml = """
    <mujoco>
        <worldbody>
            <body name="body1" pos="0 0 0">
                <joint name="a" axis="1 0 0" pos="0 0 0" type="slide"/>
                <geom name="geom1" pos="0 0 0" size="1.0"/>
                <body name="body2" pos="0 0 1">
                    <joint name="b" axis="1 0 0" pos="0 0 1" type="slide"/>
                    <geom name="geom2" pos="0 0 0" size="0.5"/>
                    <site name="site1" size="0.1"/>
                </body>
            </body>
        </worldbody>
        <actuator>
            <motor joint="a"/>
            <motor joint="b"/>
        </actuator>
    </mujoco>
    """
    model = load_model_from_xml(xml)
    sim = MjSim(model)
    sim.reset()
    sim.forward()
    # Check that xvelp starts out at zero (since qvel is zero)
    site1_xvelp = sim.data.get_site_xvelp('site1')
    np.testing.assert_allclose(site1_xvelp, np.zeros(3))
    # Push the base body and step forward to get it moving
    sim.data.ctrl[0] = 1e9
    sim.step()
    sim.forward()
    # Check that the first body has nonzero xvelp
    body1_xvelp = sim.data.get_body_xvelp('body1')
    assert not np.allclose(body1_xvelp, np.zeros(3))
    # Check that the second body has zero xvelp (still)
    body2_xvelp = sim.data.get_body_xvelp('body2')
    np.testing.assert_allclose(body2_xvelp, np.zeros(3))
    # Check that this matches the batch (gathered) getter property
    np.testing.assert_allclose(body2_xvelp, sim.data.body_xvelp[2])


def test_xvelr():  # xvelr = rotational velocity in world frame
    xml = """
    <mujoco>
        <worldbody>
            <body name="body1" pos="0 0 0">
                <joint name="a" axis="1 0 0" pos="0 0 0" type="hinge"/>
                <geom name="geom1" pos="0 0 0" size="0.3"/>
                <body name="body2" pos="0 0 1">
                    <joint name="b" axis="1 0 0" pos="0 0 0" type="hinge"/>
                    <geom name="geom2" pos="0 0 0" size="0.3"/>
                    <site name="site1" size="0.1"/>
                </body>
            </body>
        </worldbody>
        <actuator>
            <motor joint="a"/>
            <motor joint="b"/>
        </actuator>
    </mujoco>
    """
    model = load_model_from_xml(xml)
    sim = MjSim(model)
    sim.reset()
    sim.forward()
    # Check that xvelr starts out at zero (since qvel is zero)
    site1_xvelr = sim.data.get_site_xvelr('site1')
    np.testing.assert_allclose(site1_xvelr, np.zeros(3))
    # Push the base body and step forward to get it moving
    sim.data.ctrl[0] = 1e9
    sim.step()
    sim.forward()
    # Check that the first body has nonzero xvelr
    body1_xvelr = sim.data.get_body_xvelr('body1')
    assert not np.allclose(body1_xvelr, np.zeros(3))
    # Check that the second body has zero xvelr (still)
    body2_xvelr = sim.data.get_body_xvelr('body2')
    np.testing.assert_allclose(body2_xvelr, np.zeros(3))
    # Check that this matches the batch (gathered) getter property
    np.testing.assert_allclose(body2_xvelr, sim.data.body_xvelr[2])


@pytest.mark.requires_rendering
def test_rendering():
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    sim.forward()

    img, depth = sim.render(200, 200, depth=True)
    assert img.shape == (200, 200, 3)
    compare_imgs(img, 'test_rendering.freecam.png')

    depth = (depth - np.min(depth)) / (np.max(depth) - np.min(depth))
    depth = np.asarray(depth * 255, dtype=np.uint8)
    assert depth.shape == (200, 200)
    compare_imgs(depth, 'test_rendering.freecam.depth.png')

    img = sim.render(100, 100, camera_name="camera1")
    assert img.shape == (100, 100, 3)
    compare_imgs(img, 'test_rendering.camera1.png')

    img = sim.render(200, 100, camera_name="camera1")
    assert img.shape == (100, 200, 3)
    compare_imgs(img, 'test_rendering.camera1.narrow.png')

    render_context = sim.render_contexts[0]
    render_context.add_marker(size=np.array([.4, .5, .6]),
                              pos=np.array([.4, .5, .6]),
                              rgba=np.array([.7, .8, .9, 1.0]),
                              label="mark")
    img = sim.render(200, 200, camera_name="camera1")
    assert img.shape == (200, 200, 3)
    compare_imgs(img, 'test_rendering_markers.camera1.png')


@pytest.mark.requires_rendering
def test_rendering_failing():
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    sim.forward()
    sim.render(100, 100)
    render_context = sim.render_contexts[0]
    render_context.add_marker(size=np.array([.4, .5, .6]),
                              pos=np.array([.4, .5, .6]),
                              rgba=np.array([.7, .8, .9, 1.0]),
                              label="blaaaa")
    img = sim.render(200, 200, camera_name="camera1")
    assert img.shape == (200, 200, 3)
    try:
        compare_imgs(img, 'test_rendering_markers.camera1.png')
        assert False
    except Exception as e:
        pass


@pytest.mark.requires_rendering
def test_viewercontext():
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    sim.forward()
    renderer = cymj.MjRenderContext(sim, offscreen=True)
    renderer.add_marker(type=const.GEOM_SPHERE,
                        size=np.ones(3) * 0.1,
                        pos=np.zeros(3),
                        mat=np.eye(3).flatten(),
                        rgba=np.ones(4),
                        label="mark")


def test_xml_from_path():
    model = load_model_from_path("mujoco_py/tests/test.xml")
    sim = MjSim(model)
    xml = model.get_xml()
    assert xml.find("blabla") > -1, "include should be embeeded"
    assert xml.find("include") == - \
        1, "include should be parsed and not present"


def test_sensors():
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    sim.model.sensor_names
    sim.data.get_sensor("touchsensor")


@pytest.mark.requires_rendering
@pytest.mark.skipif("Darwin" not in sys.platform,
                    reason="Only Darwin code is thread safe.")
def test_concurrent_rendering():
    '''Best-effort testing that concurrent multi-threaded rendering works.
    The test has no guarantees around being deterministic, but if it fails
    you know something is wrong with concurrent rendering. If it passes,
    things are probably working.'''
    err = None
    def func(sim, event):
        event.wait()
        sim.data.qpos[:] = 0.0
        sim.forward()
        img1 = sim.render(width=40, height=40, camera_name="camera1")
        img2 = sim.render(width=40, height=40, camera_name="camera2")
        try:
            assert np.sum(img1[:]) == 23255
            assert np.sum(img2[:]) == 12007
        except Exception as e:
            nonlocal err
            err = e

    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    sim.render(100, 100)
    event = Event()
    threads = []
    for _ in range(100):
        thread = Thread(target=func, args=(sim, event))
        threads.append(thread)
        thread.start()
    event.set()
    for thread in threads:
        thread.join()
    assert err is None, "Exception: %s" % (str(err))

@pytest.mark.requires_rendering
def test_high_res():
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    sim.forward()
    img = sim.render(1000, 1000)
    img = scipy.misc.imresize(img, (200, 200, 3))
    assert img.shape == (200, 200, 3)
    compare_imgs(img, 'test_rendering.freecam.png')


@pytest.mark.skipif(sys.platform.startswith("win"), reason="This test fails on windows.")
def test_multiprocess():
    '''
    Tests for importing mujoco_py from multiple processes.
    '''
    ctx = get_context('spawn')
    processes = []
    times = 3
    queue = ctx.Queue()
    for idx in range(3):
        processes.append(ctx.Process(target=import_process, args=(queue, )))
    for p in processes:
        p.start()
    for p in processes:
        p.join()
    for _ in range(times):
        assert queue.get(), "One of processes failed."


def import_process(queue):
    try:
        from mujoco_py import builder
        mjpro_path, key_path = builder.discover_mujoco()
        builder.load_cython_ext(mjpro_path)
    except Exception as e:
        queue.put(False)
    else:
        queue.put(True)


class TestUserdata(unittest.TestCase):
    def test_userdata(self):
        xml = '''
            <mujoco>
                <size nuserdata="{}"/>
                <worldbody>
                    <body pos="0 0 0">
                        <joint type="free"/>
                        <geom type="sphere" size=".1"/>
                    </body>
                </worldbody>
            </mujoco>
        '''
        model = load_model_from_xml(xml.format(1))
        assert model.nuserdata == 1, 'bad nuserdata {}'.format(model.nuserdata)
        model = load_model_from_xml(xml.format(10))
        assert model.nuserdata == 10, 'bad nuserdata {}'.format(model.nuserdata)
        sim = MjSim(model)
        data = sim.data
        assert data.userdata[0] == 0, 'bad userdata {}'.format(data.userdata)
        data.userdata[0] = 1
        assert data.userdata[0] == 1, 'bad userdata {}'.format(data.userdata)
        # Check that we throw an assert if there's not enough userdata
        model = load_model_from_xml(xml.format(0))
        with self.assertRaises(AssertionError):
            model.set_userdata_names(['foo'])
        # Doesn't throw assert
        model = load_model_from_xml(xml.format(1))
        model.set_userdata_names(['foo'])
        with self.assertRaises(AssertionError):
            model.set_userdata_names(['foo', 'bar'])
