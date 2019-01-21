import subprocess
import glob
import os.path
import mujoco_py
import sys
import pytest
mujoco_py_root = os.path.dirname(os.path.dirname(mujoco_py.__file__))


@pytest.mark.requires_rendering
@pytest.mark.requires_glfw
def test_examples():
    scripts = glob.glob("%s/examples/*.py" % mujoco_py_root)
    env = os.environ.update({'TESTING': 'true'})
    assert len(scripts) > 0, 'No example scripts found!'
    for tutorial_script in scripts:
        if tutorial_script.find("mjvive") > -1:
            continue
        print("Executing %s" % tutorial_script)

        subprocess.check_call([sys.executable, tutorial_script], env=env)
