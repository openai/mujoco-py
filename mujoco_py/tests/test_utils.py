import os
from os.path import join, expanduser, abspath

from mujoco_py import utils

def test_discover_mujoco():
    ''' Testing MuJoCo discovery '''

    # Defaults
    mjpro_path, key_path = utils.discover_mujoco()
    assert mjpro_path == join(expanduser('~'), '.mujoco', 'mjpro150')
    assert key_path == join(expanduser('~'), '.mujoco', 'mjkey.txt')

    # Override MUJOCO_PY_MUJOCO_PATH
    override_path = join('my','mujoco','path')
    os.environ[utils._MUJOCO_PY_MUJOCO_PATH] = override_path
    mjpro_path, key_path = utils.discover_mujoco()
    assert mjpro_path == abspath(join(override_path, 'mjpro150'))
    assert key_path == abspath(join(override_path, 'mjkey.txt'))

    # Override MUJOCO_LICENSE_KEY
    override_key = join('path', 'to', 'my', 'key.txt')
    os.environ[utils._MUJOCO_LICENSE_KEY] = override_key
    _, key_path = utils.discover_mujoco()
    assert key_path == abspath(override_key)

    # Override MUJOCO_PY_MJPRO_PATH
    override_mjpro = join('path', 'to', 'my', 'mjpro')
    os.environ[utils._MUJOCO_PY_MJPRO_PATH] = override_mjpro
    mjpro_path, _ = utils.discover_mujoco()
    assert mjpro_path == abspath(override_mjpro)
