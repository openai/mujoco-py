import subprocess
import sys
import os
import pytest


@pytest.mark.skipif(sys.platform.startswith("win"), reason="This test fails on windows.")
def test_gen_wrappers():
    # Verifies that gen_wrappers can be executed.
    if sys.platform.startswith("win"):
        fname = "C:\generated_wrappers.pxi"
    else:
        fname = "/tmp/generated_wrappers.pxi"
    subprocess.check_call([sys.executable,
                           os.path.join("scripts", "gen_wrappers.py"), fname])
