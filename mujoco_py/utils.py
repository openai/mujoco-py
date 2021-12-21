import sys
import os
import copy
from os.path import join, expanduser, exists

import numpy as np

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
    - mujoco_path (str): Path to MuJoCo 2.1 directory.
    """
    mujoco_path = os.getenv('MUJOCO_PY_MUJOCO_PATH')
    if not mujoco_path:
        mujoco_path = join(expanduser('~'), '.mujoco', 'mujoco210')

    # We get lots of github issues that seem to be missing these
    # so check that mujoco is really there and raise errors if not.
    if not exists(mujoco_path):
        message = MISSING_MUJOCO_MESSAGE.format(mujoco_path)
        print(message, file=sys.stderr)
        raise Exception(message)

    return mujoco_path
