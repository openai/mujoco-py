#!/usr/bin/env python3
"""
Example how substep_callback can be used to detect contacts/penetrations that
are not visible in between steps.

In this example a sphere pushes a cylinder, which then slides away.
The entirety of the contact and separation happens in substeps.

In between steps there is always zero contacts, so the push is not observed.

We use a substep_callback to get a sum of the external forces on the cylinder.
"""
from mujoco_py import load_model_from_xml, MjSim

MODEL_XML = """
<mujoco>
    <size nuserdata="1"/>
    <worldbody>
        <body name="robot" pos="0 0 0">
            <geom rgba="1 0 0 1" size="0.15" type="sphere"/>
            <joint axis="1 0 0" damping="1" name="j" type="slide"/>
        </body>
        <body name="cylinder" pos=".5 0 0">
            <geom size="0.15 0.15" type="cylinder"/>
            <joint axis="1 0 0" damping="10" type="slide"/>
        </body>
    </worldbody>
    <actuator>
        <position joint="j" kp="2000"/>
    </actuator>
</mujoco>
"""

fn = '''
    #define SQUARE(a) (a * a)
    void fun(const mjModel* m, mjData* d) {
        for (int i = d->ne; i < d->nefc; i++) {
            pos_sum += SQUARE(d->efc_pos[i]);
        }
    }
'''

sim = MjSim(load_model_from_xml(MODEL_XML), nsubsteps=50,
            substep_callback=fn, userdata_names=['pos_sum'])
t = 0
while t < 10:
    t += 1
    sim.data.ctrl[0] = .2
    print('t', t)
    sim.step()
    # verify that there are no contacts visible between steps
    assert sim.data.ncon == 0, 'No contacts should be detected here'
    # verify that contacts (and penetrations) are visible to substep_callback
    if t > 1:
        assert sim.data.get_userdata('pos_sum') > 0  # detected collision
