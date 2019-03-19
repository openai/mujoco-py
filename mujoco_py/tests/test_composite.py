"""
Test whether library can parse composite object definitions - new in MuJoCo 2.0
"""

from mujoco_py import load_model_from_xml


PARTICLE_XML = """
<mujoco>
    <worldbody>
        <composite type="particle" count="10 10 10" spacing="0.07" offset="0 0 1">
            <geom size=".02" rgba=".8 .2 .1 1"/>
        </composite>
    </worldbody>
</mujoco>
"""


def test_load_particle():
    load_model_from_xml(PARTICLE_XML)
