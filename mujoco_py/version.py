__all__ = ['__version__', 'get_version']

version_info = (2, 0, 2, 13)
# format:
# ('mujoco_major', 'mujoco_minor', 'mujoco_py_major', 'mujoco_py_minor')


def get_version():
    """ Returns the version as a human-format string. """
    return '%d.%d.%d.%d' % version_info


__version__ = get_version()
