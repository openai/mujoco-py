import os
import sys
from mujoco_py.utils import discover_mujoco, MISSING_KEY_MESSAGE
from os.path import exists
from mujoco_py.utils import load_dynamic_ext, remove_mujoco_build
import subprocess
from subprocess import CalledProcessError
import glob


class ignore_mujoco_warnings:
    """
    Class to turn off mujoco warning exceptions within a scope. Useful for
    large, vectorized rollouts.
    """

    def __enter__(self):
        self.prev_user_warning = cymj.get_warning_callback()
        cymj.set_warning_callback(user_warning_ignore_exception)
        return self

    def __exit__(self, type, value, traceback):
        cymj.set_warning_callback(self.prev_user_warning)


class MujocoException(Exception):
    pass


def user_warning_raise_exception(warn_bytes):
    '''
    User-defined warning callback, which is called by mujoco on warnings.
    Here we have two primary jobs:
        - Detect known warnings and suggest fixes (with code)
        - Decide whether to raise an Exception and raise if needed
    More cases should be added as we find new failures.
    '''
    # TODO: look through test output to see MuJoCo warnings to catch
    # and recommend. Also fix those tests
    warn = warn_bytes.decode()  # Convert bytes to string
    if 'Pre-allocated constraint buffer is full' in warn:
        raise MujocoException(warn + 'Increase njmax in mujoco XML')
    if 'Pre-allocated contact buffer is full' in warn:
        raise MujocoException(warn + 'Increase njconmax in mujoco XML')
    # This unhelpfully-named warning is what you get if you feed MuJoCo NaNs
    if 'Unknown warning type' in warn:
        raise MujocoException(warn + 'Check for NaN in simulation.')
    raise MujocoException('Got MuJoCo Warning: {}'.format(warn))


def user_warning_ignore_exception(warn_bytes):
    pass


def find_key():
    ''' Try to find the key file, if missing, print out a big message '''
    if exists(key_path):
        return
    print(MISSING_KEY_MESSAGE.format(key_path), file=sys.stderr)


def activate():
    functions.mj_activate(key_path)


def compile_with_multiple_attempts():
    compile_mujoco_path = os.path.join(os.path.dirname(__file__), "compile_mujoco.py")
    for attempt in range(3):
        try:
            subprocess.check_call(["python", compile_mujoco_path], timeout=150)
            so_path = os.path.join(os.path.dirname(__file__), "generated", "*.so")
            cext_so_path = glob.glob(so_path)
            assert len(cext_so_path) == 1, ("Expecting only one .so file under " + so_path)
            cext_so_path = cext_so_path[0]
            cymj = load_dynamic_ext('cymj', cext_so_path)
            return cymj
        except (CalledProcessError, TimeoutError, ImportError) as _:
            remove_mujoco_build()  # Cleans the installation.
    raise Exception("Failed to compile mujoco_py.")


# Trick to expose all mj* functions from mujoco in mujoco_py.*
class dict2(object):
    pass


cymj = compile_with_multiple_attempts()
mujoco_path, key_path = discover_mujoco()
functions = dict2()
for func_name in dir(cymj):
    if func_name.startswith("_mj"):
        setattr(functions, func_name[1:], getattr(cymj, func_name))

# Set user-defined callbacks that raise assertion with message
cymj.set_warning_callback(user_warning_raise_exception)
