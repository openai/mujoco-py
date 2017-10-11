#!/usr/bin/env python
import os
import unittest
import numpy as np
from cffi import FFI
from mujoco_py import load_model_from_xml, MjSim, cymj
from mujoco_py.builder import mjpro_path, manually_link_libraries

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

DUMMY_CDEF = '''
    extern uintptr_t hello_fn;
    extern double result;
'''

# TODO: fixed header set always prepended
# TODO: generate defines from list of fields
DUMMY_SOURCE = '''
    #include <stdio.h>
    #include <mujoco.h>
    #define my_field d->userdata[0]
''' + DUMMY_CDEF + '''
    double result;
    static void hello(const mjModel* m, mjData* d) {
        printf("hello\\n");
    }
    uintptr_t hello_fn = (uintptr_t) hello;
'''


class TestSubstep(unittest.TestCase):
    def build_stubstep(self):
        ffibuilder = FFI()
        ffibuilder.cdef(DUMMY_CDEF)
        # TODO: randomize library name to prevent conflicts
        # TODO: build library name is some directory in mujoco-py
        # TODO: time library building and note how long it should take
        ffibuilder.set_source("_substep_udd", DUMMY_SOURCE,
                              include_dirs=[MJ_INCLUDE, np.get_include],
                              library_dirs=[MJ_BIN],
                              libraries=['mujoco150'])
        ffibuilder.compile(verbose=True)
        from __pycache__._todo_randomize import lib  # noqa, import compiled function
        return lib.hello_fn

    def test_substep(self):
        # TODO: desired interface
        # TODO: check for room in userdata for fields
        sim = MjSim(load_model_from_xml(XML))
        substep_fn = self.build_stubstep()
        sim.set_substep_fn(substep_fn)


if __name__ == '__main__':
    unittest.main()
