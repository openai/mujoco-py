#!/usr/bin/env python
import unittest
import numpy as np
from mujoco_py import load_model_from_xml, MjSim, functions


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

INCREMENT_FN = '''
    void fun(const mjModel* m, mjData* d) {
        d->userdata[0] += 1;
    }
'''


class TestSubstep(unittest.TestCase):
    def test_hello(self):
        fn = '''
            #include <stdio.h>
            void fun(const mjModel* m, mjData* d) {
                printf("hello");
            }
        '''
        sim = MjSim(load_model_from_xml(XML.format(nuserdata=0)),
                    substep_callback=fn)
        sim.step()  # should print 'hello'

    def test_increment(self):
        sim = MjSim(load_model_from_xml(XML.format(nuserdata=1)),
                    substep_callback=INCREMENT_FN)
        self.assertEqual(sim.data.userdata[0], 0)
        sim.step()  # should increment userdata[0]
        self.assertEqual(sim.data.userdata[0], 1)
        # Test again with different nsubsteps, reuse model
        sim = MjSim(sim.model, nsubsteps=7, substep_callback=INCREMENT_FN)
        self.assertEqual(sim.data.userdata[0], 0)
        sim.step()  # should increment userdata[0] 7 times
        self.assertEqual(sim.data.userdata[0], 7)

    def test_sum_ctrl(self):
        fn = '''
            void fun(const mjModel* m, mjData* d) {
                for (int i = 0; i < m->nu; i++) {
                    my_sum += d->ctrl[i];
                }
            }
        '''
        sim = MjSim(load_model_from_xml(XML.format(nuserdata=1)),
                    substep_callback=fn,
                    userdata_names=['my_sum'])
        self.assertEqual(sim.data.userdata[0], 0)
        sim.step()  # should set userdata[0] to sum of controls
        self.assertEqual(sim.data.userdata[0], 0)
        sim.data.ctrl[0] = 12.
        sim.data.ctrl[1] = .34
        sim.step()
        self.assertEqual(sim.data.userdata[0], 12.34)
        sim.step()
        self.assertEqual(sim.data.userdata[0], 24.68)

    def test_penetrations(self):
        # Test that we capture the max penetration of a falling sphere
        # Given a single step with many substeps
        xml = '''
            <mujoco>
                <size nuserdata="1"/>
                <worldbody>
                    <body pos="0 0 1">
                        <joint type="free"/>
                        <geom size=".1"/>
                    </body>
                    <geom type="plane" size="1 1 1"/>
                </worldbody>
            </mujoco>
        '''
        # NOTE: penetration is negative distance in contacts
        fn = '''
            #define MIN(a, b) (a < b ? a : b)
            void fun(const mjModel* m, mjData* d) {
                for (int i = 0; i < d->ncon; i++) {
                    min_contact_dist = MIN(min_contact_dist, d->contact[i].dist);
                }
            }
        '''
        sim = MjSim(load_model_from_xml(xml), nsubsteps=1000,
                    substep_callback=fn, userdata_names=['min_contact_dist'])
        self.assertEqual(sim.data.userdata[0], 0)
        sim.step()
        self.assertEqual(sim.data.ncon, 1)  # Assert we have a contact
        self.assertLess(sim.data.contact[0].dist, 0)  # assert penetration
        # Assert that min penetration is much less than current penetration
        self.assertLess(sim.data.userdata[0], 10 * sim.data.contact[0].dist)

    def test_reuse(self):
        ''' Test that we can re-use a substep_callback between sims '''
        fn = '''
            #define MAX(a, b) (a > b ? a : b)
            void fun(const mjModel* m, mjData* d) {
                for (int i = 0; i < m->nu; i++) {
                    ctrl_max = MAX(ctrl_max, d->ctrl[i]);
                }
            }
        '''
        sim = MjSim(load_model_from_xml(XML.format(nuserdata=1)),
                    substep_callback=fn, userdata_names=['ctrl_max'])
        sim.step()
        self.assertEqual(sim.data.userdata[0], 0)
        sim.data.ctrl[:] = [1, 2]
        sim.step()
        self.assertEqual(sim.data.userdata[0], 2)
        sim2 = MjSim(sim.model, substep_callback=sim.substep_callback_ptr)
        sim2.step()
        self.assertEqual(sim2.data.userdata[0], 0)
        sim2.data.ctrl[:] = [.1, .2]
        sim2.step()
        self.assertEqual(sim2.data.userdata[0], .2)

    def test_mjfunctions(self):
        # Test calling mujoco function from within callback
        fn = '''
            void fun(const mjModel *m, mjData *d) {
                // set userdata to rotation matrix for body_xquat[2]
                mju_quat2Mat(d->userdata, &(d->xquat[4 * 2]));
            }
        '''
        sim = MjSim(load_model_from_xml(XML.format(nuserdata=9)),
                    substep_callback=fn)
        sim.data.ctrl[:] = [9, 13]
        for _ in range(30):
            sim.step()
        sim.substep_callback()
        mat = np.zeros(9, dtype=np.float64)
        functions.mju_quat2Mat(mat, sim.data.body_xquat[2])
        np.testing.assert_array_equal(sim.data.userdata, mat)


if __name__ == '__main__':
    unittest.main()
