#!/usr/bin/env python
import unittest
from mujoco_py import load_model_from_xml, MjSim

XML = """<mujoco>
    <worldbody>
        <body pos="0 0 0">
            <joint name="j1" type="hinge" axis="1 0 0"/>
            <geom type="sphere" size=".5"/>
            <body pos="0 0 1">
                <joint name="j2" type="hinge" axis="1 0 0"/>
                <geom type="sphere" size=".1"/>
            </body>
        </body>
    </worldbody>
    <actuator>
        <position name="a1" joint="j1" kp="10000"/>
        <position name="a2" joint="j2" kp="10000"/>
        </actuator>
</mujoco>
"""


class TestSubstep(unittest.TestCase):
    def test_substep(self):
        sim = MjSim(load_model_from_xml(XML))


if __name__ == '__main__':
    unittest.main()
