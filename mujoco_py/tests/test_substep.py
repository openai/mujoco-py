#!/usr/bin/env python
import unittest
from mujoco_py import load_model_from_xml, MjSim
from mujoco_py import MjViewer

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
            void generic(const mjModel* m, mjData* d) {
                for (int i = 0; i < d->ncon; i++) {
                    min_contact_dist = MIN(min_contact_dist, d->contact[i].dist);
                }
            }
        '''
        fields = ['min_contact_dist']
        sim = MjSim(load_model_from_xml(xml),
                    nsubsteps=1000,
                    substep_udd_fn=fn,
                    substep_udd_fields=fields)
        self.assertEqual(sim.data.userdata[0], 0)
        sim.step()
        self.assertEqual(sim.data.ncon, 1)  # Assert we have a contact
        self.assertLess(sim.data.contact[0].dist, 0)  # assert penetration
        # Assert that min penetration is much less than current penetration
        self.assertLess(sim.data.userdata[0], 10 * sim.data.contact[0].dist)

    def test_reuse(self):
        ''' Test that we can re-use a substep_udd_fn between sims '''
        fn = '''
            #define MAX(a, b) (a > b ? a : b)
            void generic(const mjModel* m, mjData* d) {
                for (int i = 0; i < m->nu; i++) {
                    ctrl_max = MAX(ctrl_max, d->ctrl[i]);
                }
            }
        '''
        sim = MjSim(load_model_from_xml(XML.format(nuserdata=1)),
                    substep_udd_fn=fn,
                    substep_udd_fields=['ctrl_max'])
        sim.step()
        self.assertEqual(sim.data.userdata[0], 0)
        sim.data.ctrl[:] = [1, 2]
        sim.step()
        self.assertEqual(sim.data.userdata[0], 2)
        sim2 = MjSim(sim.model,
                     substep_udd_fn=sim.substep_udd_fn,
                     substep_udd_fields=sim.substep_udd_fields)
        sim2.step()
        self.assertEqual(sim2.data.userdata[0], 0)
        sim2.data.ctrl[:] = [.1, .2]
        sim2.step()
        self.assertEqual(sim2.data.userdata[0], .2)


if __name__ == '__main__':
    unittest.main()
