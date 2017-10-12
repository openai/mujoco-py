import subprocess
import sys
import os
import pytest
import mujoco_py


@pytest.mark.skipif(sys.platform.startswith("win"), reason="This test fails on windows.")
def test_gen_wrappers():
    # Verifies that gen_wrappers can be executed.
    fname = "/tmp/generated_wrappers.pxi"
    subprocess.check_call([sys.executable,
                           os.path.join("scripts", "gen_wrappers.py"), fname])
    return fname


@pytest.mark.skipif(sys.platform.startswith("win"), reason="This test fails on windows.")
def test_deterministic():
    # Verifies that gen_wrappers is deterministic
    fname = test_gen_wrappers()
    with open(fname) as f:
        s = f.read()
    fname = test_gen_wrappers()
    with open(fname) as f:
        assert f.read() == s, 'Generate wrappers returned different result'


@pytest.mark.skipif(sys.platform.startswith("win"), reason="This test fails on windows.")
def test_generated():
    # Verifies generated wrappers match checked-in wrappers
    fname = test_gen_wrappers()
    with open(fname) as f:
        generated_str = f.read()
    gname = os.path.join(os.path.dirname(mujoco_py.cymj.__file__), 'wrappers.pxi')
    with open(gname) as f:
        checkedin_str = f.read()
    assert generated_str == checkedin_str, 'Generated wrappers do not match'
