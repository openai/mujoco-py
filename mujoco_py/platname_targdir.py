import sys
if sys.platform.startswith("darwin"):
    platname = "osx"
elif sys.platform.startswith("linux"):
    platname = "linux"
elif sys.platform.startswith("win"):
    platname = "win"
targdir = "mujoco_%s"%platname

