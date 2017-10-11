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

DUMMY_CDEF = '''
    extern uintptr_t hello_fn;
'''

# TODO: fixed header set always prepended
# TODO: generate defines from list of fields
DUMMY_SOURCE = '''
    #include <stdio.h>
    #include <mujoco.h>
    #define my_field d->userdata[0]
''' + DUMMY_CDEF + '''
    static void hello(const mjModel* m, mjData* d) {
        printf("hello\\n");
    }
    uintptr_t hello_fn = (uintptr_t) hello;
'''


class TestSubstep(unittest.TestCase):
    def build_stubstep(self):
        ffibuilder = FFI()
        ffibuilder.cdef(DUMMY_CDEF)
        # TODO: time library building and note how long it should take
        name = '_substep_udd_'
        name += ''.join(choice(ascii_lowercase) for _ in range(15))
        ffibuilder.set_source(name, DUMMY_SOURCE,
                              include_dirs=[MJ_INCLUDE],
                              library_dirs=[MJ_BIN],
                              libraries=['mujoco150'])
        library_path = ffibuilder.compile(verbose=True)
        fixed_library_path = manually_link_libraries(mjpro_path, library_path)
        module = load_dynamic_ext(name, fixed_library_path)
        # Now that the module is loaded into memory, we can actually delete it
        for f in glob.glob(name + '*'):
            os.remove(f)
        return module.lib.hello_fn

    def test_substep(self):
        # TODO: desired interface
        # TODO: check for room in userdata for fields
        sim = MjSim(load_model_from_xml(XML))
        substep_fn = self.build_stubstep()
        sim.set_substep_udd_fn(substep_fn)
        sim.step()


if __name__ == '__main__':
    unittest.main()
