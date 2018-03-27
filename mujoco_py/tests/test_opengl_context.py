import pytest
from mujoco_py import MjSim, load_model_from_xml, MjRenderContext, GlfwContext
from mujoco_py.tests.utils import compare_imgs

BASIC_MODEL_XML = """
<mujoco>
    <asset>
        <texture name="t1" width="33" height="36" type="2d" builtin="flat" />
        <texture name="t2" width="34" height="39" type="2d" builtin="flat" />
        <texture name="t3" width="31" height="37" type="2d" builtin="flat" />
        <texture name="t4" width="38" height="34" type="2d" builtin="flat" />
        <material name="m1" texture="t1" />
        <material name="m2" texture="t2" />
        <material name="m3" texture="t3" />
        <material name="m4" texture="t4" />
    </asset>
    <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 5" dir="0 0 -1" />
        <camera name="topcam" pos="0 0 2.5" zaxis="0 0 1" />
        <geom name="g1" pos="-0.5 0.5 0" type="box" size="0.4 0.4 0.1" rgba="1 1 1 1" material="m1" />
        <geom name="g2" pos="0.5 0.5 0" type="box" size="0.4 0.4 0.1" rgba="1 1 1 1" material="m2" />
        <geom name="g3" pos="0.5 -0.5 0" type="box" size="0.4 0.4 0.1" rgba="1 1 1 1" material="m3" />
        <geom name="g4" pos="-0.5 -0.5 0" type="box" size="0.4 0.4 0.1" rgba="1 1 1 1" material="m4" />
        <!-- The following boxes will show up in reflections -->
        <geom pos="0.5 0.5 4" type="box" size="0.4 0.4 0.1" rgba="1 1 1 1" />
        <geom pos="0.5 -0.5 4" type="box" size="0.4 0.4 0.1" rgba="1 1 1 1" />
        <geom pos="-0.5 -0.5 4" type="box" size="0.4 0.4 0.1" rgba="1 1 1 1" />
        <geom pos="-0.5 0.5 4" type="box" size="0.4 0.4 0.1" rgba="1 1 1 1" />
    </worldbody>
</mujoco>
"""

@pytest.mark.requires_rendering
def test_glfw_context():
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    sim.forward()

    render_context = MjRenderContext(sim, offscreen=True, opengl_backend='glfw')
    assert len(sim.render_contexts) == 1
    assert sim.render_contexts[0] is render_context
    assert isinstance(render_context.opengl_context, GlfwContext)

    compare_imgs(sim.render(201, 205, camera_name="topcam"), 'test_glfw_context.png')
    assert len(sim.render_contexts) == 1
    assert sim.render_contexts[0] is render_context
