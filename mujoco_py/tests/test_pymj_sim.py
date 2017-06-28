import subprocess
import sys
import mujoco_py
import pytest
import os


def test_import_mujoco_py_as_cymj_with_shim():
    pip = 'pip3' if sys.executable.endswith('3') else 'pip'
    cmd = [pip, 'install', '--verbose', os.path.join('vendor', 'pymj_shim')]
    return_code = subprocess.check_call(cmd)
    assert return_code == 0, "Failed to " + (" ".join(cmd))
    with pytest.warns(UserWarning):
        import pymj
        assert pymj.MjSim == mujoco_py.MjSim
        assert pymj.__version__ == mujoco_py.__version__
