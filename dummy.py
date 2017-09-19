#!/usr/bin/env python
import tracemalloc
from mujoco_py import MjSim, load_model_from_xml, set_act_gain_callback_fn

tracemalloc.start()
xml = '''
<mujoco>
    <worldbody>
        <body name="body1" pos="0 0 0">
            <joint axis="1 0 0" name="a" pos="0 0 0" type="hinge"/>
            <geom pos="0 0 0" size="1.0"/>
        </body>
    </worldbody>
    <actuator>
        <general joint="a" gaintype="user"/>
    </actuator>
</mujoco>
'''
sim = MjSim(load_model_from_xml(xml))

def p(model, data, id_):
    print(model, data, id_)
    return None

sim.step()

for lineno in tracemalloc.take_snapshot().statistics('lineno')[:10]:
    print(lineno)


set_act_gain_callback_fn(p)

sim.step()
