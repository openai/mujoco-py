#!/usr/bin/env python
import os
import glob
import unittest
from random import choice
from shutil import move
from string import ascii_lowercase
from cffi import FFI
from mujoco_py import load_model_from_xml, MjSim, cymj
from mujoco_py.builder import mjpro_path, manually_link_libraries, load_dynamic_ext

MJ_INCLUDE = os.path.join(mjpro_path, 'include')
MJ_BIN = os.path.join(mjpro_path, 'bin')
GENERATED_DIR = os.path.dirname(cymj.__file__)

XML = '''
<mujoco>
    <worldbody>
        <body pos="0 0 0">
            <joint name="j" type="hinge" axis="1 0 0"/>
            <geom type="sphere" size=".5"/>
        </body>
    </worldbody>
    <actuator>
        <position name="a" joint="j" kp="100"/>
    </actuator>
</mujoco>
'''


GENERIC_FN = '''
    #include <stdio.h>
    static void generic(const mjModel* m, mjData* d) {
        printf("hello\\n");
    }
'''


class TestSubstep(unittest.TestCase):
    def test_substep(self):
        # TODO: desired interface
        # TODO: check for room in userdata for fields
        sim = MjSim(load_model_from_xml(XML), substep_udd_fn=GENERIC_FN)
        sim.step()


if __name__ == '__main__':
    unittest.main()
