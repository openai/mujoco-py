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


HELLO_FN = '''
    #include <stdio.h>
    void generic(const mjModel* m, mjData* d) {
        printf("hello\\n");
        printf("%d\\n", m->nuserdata);
    }
'''

SINGLE_FN = '''
    void generic(const mjModel* m, mjData* d) {
        d->userdata[0] = 1;
    }
'''


class TestSubstep(unittest.TestCase):
    def test_hello(self):
        # TODO: desired interface
        sim = MjSim(load_model_from_xml(XML), substep_udd_fn=HELLO_FN)
        sim.step()  # should print 'hello'

    def test_single(self):
        sim = MjSim(load_model_from_xml(XML), substep_udd_fn=SINGLE_FN)
        self.assertEqual(sim.data.userdata[0], 0)
        sim.step()  # should print 'hello'
        self.assertEqual(sim.data.userdata[0], 1)


if __name__ == '__main__':
    unittest.main()
