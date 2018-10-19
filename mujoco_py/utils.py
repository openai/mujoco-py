import sys
import os
import copy
from os.path import join, expanduser, exists

import numpy as np

MISSING_KEY_MESSAGE = '''
You appear to be missing a License Key for mujoco.  We expected to find the
file here: {}

You can get licenses at this page:

    https://www.roboti.us/license.html

If python tries to activate an invalid license, the process will exit.
'''

MISSING_MJPRO_MESSAGE = '''
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
    - mjpro_path (str): Path to MuJoCo Pro 1.50 directory.
    - key_path (str): Path to the MuJoCo license key.
    """
    key_path = os.getenv('MUJOCO_PY_MJKEY_PATH')
    if not key_path:
        key_path = join(expanduser('~'), '.mujoco', 'mjkey.txt')
    mjpro_path = os.getenv('MUJOCO_PY_MJPRO_PATH')
    if not mjpro_path:
        mjpro_path = join(expanduser('~'), '.mujoco', 'mjpro150')

    # We get lots of github issues that seem to be missing these
    # so check that mujoco is really there and raise errors if not.
    if not exists(mjpro_path):
        message = MISSING_MJPRO_MESSAGE.format(mjpro_path)
        print(message, file=sys.stderr)
        raise Exception(message)
    if not exists(key_path):
        message = MISSING_KEY_MESSAGE.format(key_path)
        print(message, file=sys.stderr)
        raise Exception(message)

    return (mjpro_path, key_path)
