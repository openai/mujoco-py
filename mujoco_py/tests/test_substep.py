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
    <size nuserdata="{nuserdata}"/>
    <worldbody>
        <body pos="0 0 0">
            <joint name="j1" type="hinge" axis="1 0 0"/>
            <geom type="sphere" size=".5"/>
            <body pos="0 0 1">
                <joint name="j2" type="hinge" axis="1 0 0"/>
                <geom type="sphere" size=".5"/>
            </body>
        </body>
    </worldbody>
    <actuator>
        <position joint="j1" kp="100"/>
        <position joint="j2" kp="100"/>
    </actuator>
</mujoco>
'''

COMPLEX_XML = '''
<mujoco>
    <size nuserdata="1"/>
    <worldbody>
        <body pos="0 0 0">
            <joint name="j1" type="hinge" axis="1 0 0"/>
            <geom type="sphere" size=".5"/>
            <body pos="0 0 1">
                <joint name="j2" type="hinge" axis="1 0 0"/>
                <geom type="sphere" size=".5"/>
            </body>
        </body>
    </worldbody>
    <actuator>
        <position joint="j1" kp="100"/>
        <position joint="j2" kp="100"/>
    </actuator>
</mujoco>
'''


HELLO_FN = '''
    #include <stdio.h>
    void generic(const mjModel* m, mjData* d) {
        printf("hello");
    }
'''

INCREMENT_FN = '''
    void generic(const mjModel* m, mjData* d) {
        d->userdata[0] += 1;
    }
'''


class TestSubstep(unittest.TestCase):
    def test_hello(self):
        sim = MjSim(load_model_from_xml(XML.format(nuserdata=0)),
                    substep_udd_fn=HELLO_FN)
        sim.step()  # should print 'hello'

    def test_increment(self):
        sim = MjSim(load_model_from_xml(XML.format(nuserdata=1)),
                    substep_udd_fn=INCREMENT_FN)
        self.assertEqual(sim.data.userdata[0], 0)
        sim.step()  # should increment userdata[0]
        self.assertEqual(sim.data.userdata[0], 1)
        # Test again with different nsubsteps, reuse model
        sim = MjSim(sim.model, nsubsteps=7, substep_udd_fn=INCREMENT_FN)
        self.assertEqual(sim.data.userdata[0], 0)
        sim.step()  # should increment userdata[0] 7 times
        self.assertEqual(sim.data.userdata[0], 7)

    def test_sum_ctrl(self):
        fn = '''
            void generic(const mjModel* m, mjData* d) {
                for (int i = 0; i < m->nu; i++) {
                    my_sum += d->ctrl[i];
                }
            }
        '''
        sim = MjSim(load_model_from_xml(XML.format(nuserdata=1)),
                    substep_udd_fn=fn,
                    substep_udd_fields=['my_sum'])
        self.assertEqual(sim.data.userdata[0], 0)
        sim.step()  # should set userdata[0] to sum of controls
        self.assertEqual(sim.data.userdata[0], 0)
        sim.data.ctrl[0] = 12.
        sim.data.ctrl[1] = .34
        sim.step()
        self.assertEqual(sim.data.userdata[0], 12.34)
        sim.step()
        self.assertEqual(sim.data.userdata[0], 24.68)

    def test_nuserdata_assert(self):
        model = load_model_from_xml(XML.format(nuserdata=0))
        MjSim(model, substep_udd_fn=HELLO_FN)
        with self.assertRaises(AssertionError):
            MjSim(model,
                  substep_udd_fn=INCREMENT_FN,
                  substep_udd_fields=['foo'])
        # Doesn't throw assert
        model = load_model_from_xml(XML.format(nuserdata=1))
        MjSim(model,
              substep_udd_fn=INCREMENT_FN,
              substep_udd_fields=['foo'])
        with self.assertRaises(AssertionError):
            MjSim(model,
                  substep_udd_fn=INCREMENT_FN,
                  substep_udd_fields=['foo', 'bar'])


if __name__ == '__main__':
    unittest.main()
