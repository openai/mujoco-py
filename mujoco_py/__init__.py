from .config import init_config, get_key_path
init_config()

from .mjviewer import MjViewer
from .mjcore import MjModel
from .mjcore import register_license
from .mjconstants import *
from .platname_targdir import targdir

register_license(get_key_path())
