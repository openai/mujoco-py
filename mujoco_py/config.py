import distutils.version
import numpy
import os
import sys

from mujoco_py import error

key_path = None
mjpro_path = None

def init():
    global key_path, mjpro_path

    key_path = os.environ.get('MUJOCO_PY_MJKEY_PATH')
    if key_path and not os.path.exists(key_path):
        raise error.MujocoDependencyError('MUJOCO_PY_MJKEY_PATH path does not exist: {}'.format(key_path))

    mjpro_path = os.environ.get('MUJOCO_PY_MJPRO_PATH')
    if mjpro_path and not os.path.exists(mjpro_path):
        raise error.MujocoDependencyError('MUJOCO_PY_MJPRO_PATH path does not exist: {}'.format(mjpro_path))

    default_key_path = os.path.expanduser('~/.mujoco/mjkey.txt')
    default_mjpro_path = os.path.expanduser('~/.mujoco/mjpro131')
    if not key_path and os.path.exists(default_key_path):
        key_path = default_key_path
    if not mjpro_path and os.path.exists(default_mjpro_path):
        mjpro_path = default_mjpro_path

    if not key_path and not mjpro_path:
        raise error.MujocoDependencyError('To use MuJoCo, you need to either populate ~/.mujoco/mjkey.txt and ~/.mujco/mjpro131, or set the MUJOCO_PY_MJKEY_PATH and MUJOCO_PY_MJPRO_PATH environment variables appropriately. Follow the instructions on https://github.com/openai/mujoco-py for where to obtain these.')
    elif not key_path:
        raise error.MujocoDependencyError('Found your MuJoCo binaries but not license key. Please put your key into ~/.mujoco/mjkey.txt or set MUJOCO_PY_MJKEY_PATH. Follow the instructions on https://github.com/openai/mujoco-py for setup.')
    elif not mjpro_path:
        raise error.MujocoDependencyError('Found your MuJoCo license key but not binaries. Please put your binaries into ~/.mujoco/mjpro131 or set MUJOCO_PY_MJPRO_PATH. Follow the instructions on https://github.com/openai/mujoco-py for setup.')

    check_mujoco_version()
    check_numpy_version()

def check_mujoco_version():
    mjpro = os.path.basename(mjpro_path)
    if mjpro != 'mjpro131':
        raise error.MujocoDependencyError("We expected your MUJOCO_PY_MJPRO_PATH final directory to be 'mjpro131', but you provided: {} ({}). MuJoCo often changes in incompatible ways between versions, so you must use MuJoCo 1.31. If you're using MuJoCo 1.31 but changed the directory name, simply change the name back.".format(mjpro, mjpro_path))

def check_numpy_version():
    if distutils.version.StrictVersion(numpy.__version__) < distutils.version.StrictVersion('1.10.4'):
        raise error.MujocoDependencyError('You are running with numpy {}, but you must use >= 1.10.4. (In particular, earlier versions of numpy have been seen to cause mujoco-py to return different results from later ones.)'.format(numpy.__version__, '1.10.4'))
