import pytest
from mujoco_py import MjSim, load_model_from_xml
from mujoco_py.modder import MaterialModder, TextureModder
from mujoco_py.tests.utils import compare_imgs
import numpy as np

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
def test_textures():
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    sim.forward()

    compare_imgs(sim.render(201, 205, camera_name="topcam"),
                 'test_textures.premod.png')

    random_state = np.random.RandomState(0)
    modder = TextureModder(sim, random_state=random_state)
    modder.whiten_materials()
    modder.whiten_materials(['g1', 'g2'])

    modder.set_rgb('g1', (255, 0, 0))
    modder.set_rgb('g2', (0, 255, 0))
    modder.set_rgb('g3', (0, 0, 255))
    modder.set_rgb('g4', (255, 0, 255))
    compare_imgs(sim.render(201, 205, camera_name="topcam"),
                 'test_textures.rgb.png')

    modder.set_checker('g1', (255, 0, 0), (0, 255, 0))
    modder.set_gradient('g2', (0, 255, 0), (0, 0, 255), vertical=True)
    modder.set_gradient('g3', (255, 255, 0), (0, 0, 255), vertical=False)
    modder.set_noise('g4', (0, 0, 255), (255, 0, 0), 0.1)
    compare_imgs(sim.render(201, 205, camera_name="topcam"),
                 'test_textures.variety.png')

    modder.rand_checker('g1')
    modder.rand_gradient('g2')
    modder.rand_noise('g3')
    modder.rand_rgb('g4')
    compare_imgs(sim.render(201, 205, camera_name="topcam"),
                 'test_textures.rand_specific.png')

    modder.rand_all('g1')
    modder.rand_all('g2')
    modder.rand_all('g3')
    modder.rand_all('g4')
    compare_imgs(sim.render(201, 205, camera_name="topcam"),
                 'test_textures.rand_all.png')

    modder.rand_checker('g1')
    modder.rand_checker('g2')
    modder.rand_checker('g3')
    modder.rand_checker('g4')
    mat_modder = MaterialModder(sim, random_state=random_state)
    mat_modder.rand_texrepeat('g1')
    mat_modder.rand_texrepeat('g2')
    mat_modder.rand_texrepeat('g3')
    mat_modder.rand_texrepeat('g4')
    compare_imgs(sim.render(201, 205, camera_name="topcam"),
                 'test_textures.rand_texrepeat.png')

@pytest.mark.requires_rendering
def test_materials():
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    sim.forward()

    compare_imgs(sim.render(201, 205, camera_name="topcam"),
                 'test_materials.premod.png')

    random_state = np.random.RandomState(0)
    modder = MaterialModder(sim, random_state=random_state)

    modder.set_specularity('g1', 1.0)
    modder.set_reflectance('g2', 1.0)
    modder.set_shininess('g3', 1.0)
    compare_imgs(sim.render(201, 205, camera_name="topcam"),
                 'test_materials.props.png')

    modder.rand_all('g4')
    compare_imgs(sim.render(201, 205, camera_name="topcam"),
                 'test_materials.rand_all.png')

@pytest.mark.requires_rendering
def test_multiple_sims():
    # Ensure that creating new simulators still produces good renderings.
    xml = """
    <mujoco>
        <asset>
            <texture name="t1" width="32" height="32" type="2d" builtin="flat" />
            <material name="m1" texture="t1" />
        </asset>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 5" dir="0 0 -1" />
            <camera name="topcam" pos="0 0 2.5" zaxis="0 0 1" />
            <geom name="g1" pos="0 0 0" type="box" size="1 1 0.1" rgba="1 1 1 1" material="m1" />
        </worldbody>
    </mujoco>
    """

    model = load_model_from_xml(xml)
    random_state = np.random.RandomState(0)

    for i in range(3):
        sim = MjSim(model)
        sim.forward()
        modder = TextureModder(sim, random_state=random_state)
        for j in range(2):
            modder.rand_checker('g1')
            compare_imgs(
                sim.render(201, 205, camera_name="topcam"),
                'test_multiple_sims.loop%d_%d.png' % (i, j))


@pytest.mark.requires_rendering
def test_resetting():
    # Ensure that resetting environment and creating new simulators
    # still produces good renderings.
    xml = """
    <mujoco>
        <asset>
            <texture name="t1" width="32" height="32" type="2d" builtin="flat" />
            <material name="m1" texture="t1" />
        </asset>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 5" dir="0 0 -1" />
            <camera name="topcam" pos="0 0 2.5" zaxis="0 0 1" />
            <geom name="g1" pos="0 0 0" type="{geom_type}" size="1 1 0.1" rgba="1 1 1 1" material="m1" />
        </worldbody>
    </mujoco>
    """

    def get_sim(seed):
        geom_type = ["box", "sphere"][seed % 2]
        model = load_model_from_xml(xml.format(geom_type=geom_type))
        return MjSim(model)

    random_state = np.random.RandomState(0)

    for i in range(3):
        sim = get_sim(i - 1)
        sim.forward()
        modder = TextureModder(sim, random_state=random_state)
        for j in range(2):
            modder.rand_checker('g1')
            compare_imgs(sim.render(201, 205, camera_name="topcam"),
                         'test_resetting.loop%d_%d.png' % (i, j))
