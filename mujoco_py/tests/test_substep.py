#!/usr/bin/env python
import unittest
from cffi import FFI
from mujoco_py import load_model_from_xml, MjSim

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

DUMMY_CDEF = '''
    void hello(void);
    extern uintptr_t hello_fn;
'''

DUMMY_SOURCE = '''
    #include <stdio.h>
''' + DUMMY_CDEF + '''
    void hello(void) {
        printf("hello\\n");
    }
    uintptr_t hello_fn = (uintptr_t) hello;
'''


class TestSubstep(unittest.TestCase):
    def build_stubstep(self):
        ffibuilder = FFI()
        ffibuilder.cdef(DUMMY_CDEF)
        ffibuilder.set_source("_dummy", DUMMY_SOURCE)
        ffibuilder.compile(verbose=True)
        from _dummy import lib  # noqa, import compiled function
        return lib.hello_fn

    def test_substep(self):
        sim = MjSim(load_model_from_xml(XML))
        substep_fn = self.build_stubstep()
        sim.set_substep_fn(substep_fn)


if __name__ == '__main__':
    unittest.main()
