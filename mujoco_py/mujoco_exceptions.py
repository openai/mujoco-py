import sys
from mujoco_py.utils import discover_mujoco, MISSING_KEY_MESSAGE
from os.path import exists


class ignore_mujoco_warnings:
    """
    Class to turn off mujoco warning exceptions within a scope. Useful for
    large, vectorized rollouts.
    """

    def __enter__(self):
        from mujoco_py import cymj
        self.prev_user_warning = cymj.get_warning_callback()
        cymj.set_warning_callback(user_warning_ignore_exception)
        return self

    def __exit__(self, type, value, traceback):
        from mujoco_py import cymj
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


def find_activation_key():
    ''' Try to find the key file, if missing, print out a big message '''
    mujoco_path, key_path = discover_mujoco()
    if exists(key_path):
        return
    print(MISSING_KEY_MESSAGE.format(key_path), file=sys.stderr)


