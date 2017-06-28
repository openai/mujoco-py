import warnings
import sys

warnings.warn("""\
`pymj` is deprecated and will be removed soon. Please switch to mujoco-py version 1.50.0.0 or later.
To do this, replace your dependency on `pymj` with `mujoco-py<1.50.1,>=1.50.0`,
and replace `import pymj` with `import mujoco_py`.""", UserWarning, stacklevel=2)

sys.modules[__name__] = __import__('mujoco_py')
