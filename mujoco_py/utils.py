import copy
from os.path import join, expanduser

import numpy as np



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
    key_path = join(expanduser('~'), '.mujoco', 'mjkey.txt')
    mjpro_path = join(expanduser('~'), '.mujoco', 'mjpro150')
    return (mjpro_path, key_path)
