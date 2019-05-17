import sys
import copy
from os.path import join, expanduser, exists
from importlib.machinery import ExtensionFileLoader
import os
import subprocess
from os.path import abspath, dirname, exists, join, getmtime
import shutil

import numpy as np

MISSING_KEY_MESSAGE = '''
You appear to be missing a License Key for mujoco.  We expected to find the
file here: {}

You can get licenses at this page:

    https://www.roboti.us/license.html

If python tries to activate an invalid license, the process will exit.
'''

MISSING_MUJOCO_MESSAGE = '''
You appear to be missing MuJoCo.  We expected to find the file here: {}

This package only provides python bindings, the library must be installed separately.

Please follow the instructions on the README to install MuJoCo

    https://github.com/openai/mujoco-py#install-mujoco

Which can be downloaded from the website

    https://www.roboti.us/index.html
'''


def remove_empty_lines(string):
    lines = []
    for line in string.splitlines():
        if line.strip():
            lines.append(line)
    return "\n".join(lines)


def rec_assign(node, assign):
    # Assigns values to node recursively.
    # This is neccessary to avoid overriding pointers in MuJoCo.
    for field in dir(node):
        if field.find("__") == -1 and field != 'uintptr':
            val = getattr(node, field)
            if isinstance(val, (int, bool, float, None.__class__, str)):
                setattr(node, field, assign[field])
            elif isinstance(val, np.ndarray):
                val[:] = assign[field][:]
            elif not hasattr(val, "__call__"):
                rec_assign(val, assign[field])


def rec_copy(node):
    # Recursively copies object to dictionary.
    # Applying directly copy.deepcopy causes seg fault.
    ret = {}
    for field in dir(node):
        if field.find("__") == -1:
            val = getattr(node, field)
            if isinstance(val, (int, bool, float, None.__class__, str)):
                ret[field] = val
            elif isinstance(val, np.ndarray):
                ret[field] = copy.deepcopy(val)
            elif not hasattr(val, "__call__"):
                ret[field] = rec_copy(val)
    return ret


def discover_mujoco():
    """
    Discovers where MuJoCo is located in the file system.
    Currently assumes path is in ~/.mujoco

    Returns:
    - mujoco_path (str): Path to MuJoCo 2.0 directory.
    - key_path (str): Path to the MuJoCo license key.
    """
    key_path = join(expanduser('~'), '.mujoco', 'mjkey.txt')
    mujoco_path = join(expanduser('~'), '.mujoco', 'mujoco200')

    # We get lots of github issues that seem to be missing these
    # so check that mujoco is really there and raise errors if not.
    if not exists(mujoco_path):
        message = MISSING_MUJOCO_MESSAGE.format(mujoco_path)
        print(message, file=sys.stderr)
        raise Exception(message)
    if not exists(key_path):
        message = MISSING_KEY_MESSAGE.format(key_path)
        print(message, file=sys.stderr)
        raise Exception(message)

    return mujoco_path, key_path


def load_dynamic_ext(name, path):
    ''' Load compiled shared object and return as python module. '''
    loader = ExtensionFileLoader(name, path)
    return loader.load_module()


def manually_link_libraries(mujoco_path, raw_cext_dll_path):
    ''' Used to fix mujoco library linking on Mac '''
    root, ext = os.path.splitext(raw_cext_dll_path)
    final_cext_dll_path = root + '_final' + ext

    # If someone else already built the final DLL, don't bother
    # recreating it here, even though this should still be idempotent.
    if (exists(final_cext_dll_path) and
            getmtime(final_cext_dll_path) >= getmtime(raw_cext_dll_path)):
        return final_cext_dll_path

    tmp_final_cext_dll_path = final_cext_dll_path + '~'
    shutil.copyfile(raw_cext_dll_path, tmp_final_cext_dll_path)

    mj_bin_path = join(mujoco_path, 'bin')

    # Fix the rpath of the generated library -- i lost the Stackoverflow
    # reference here
    from_mujoco_path = '@executable_path/libmujoco200.dylib'
    to_mujoco_path = '%s/libmujoco200.dylib' % mj_bin_path
    subprocess.check_call(['install_name_tool',
                           '-change',
                           from_mujoco_path,
                           to_mujoco_path,
                           tmp_final_cext_dll_path])

    from_glfw_path = 'libglfw.3.dylib'
    to_glfw_path = os.path.join(mj_bin_path, 'libglfw.3.dylib')
    subprocess.check_call(['install_name_tool',
                           '-change',
                           from_glfw_path,
                           to_glfw_path,
                           tmp_final_cext_dll_path])

    os.rename(tmp_final_cext_dll_path, final_cext_dll_path)
    return final_cext_dll_path
