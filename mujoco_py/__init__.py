import os
from mujoco_py import config
config.init()

# Ok, everything's great! Do the actual work now.
from .mjviewer import MjViewer
from .mjcore import MjModel
from .mjcore import register_license
import os
import sys
from mjconstants import *
from platname_targdir import targdir

register_license(config.key_path)
