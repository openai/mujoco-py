from mujoco_py.utils import load_dynamic_ext, remove_mujoco_build
import subprocess
from subprocess import CalledProcessError
import glob
import os
from mujoco_py.mujoco_exceptions import user_warning_raise_exception


# Trick to expose all mj* functions from mujoco in mujoco_py.*
class dict2(object):
    pass


def load_cymj():
    compile_mujoco_path = os.path.join(os.path.dirname(__file__), "compile_mujoco.py")
    for attempt in range(3):
        try:
            subprocess.check_call(["python", compile_mujoco_path], timeout=150)
            so_path = os.path.join(os.path.dirname(__file__), "generated", "*.so")
            cext_so_path = glob.glob(so_path)
            assert len(cext_so_path) == 1, ("Expecting only one .so file under " + so_path)
            cext_so_path = cext_so_path[0]
            cymj = load_dynamic_ext('cymj', cext_so_path)
            # Set user-defined callbacks that raise assertion with message
            cymj.set_warning_callback(user_warning_raise_exception)
            return cymj

        except (CalledProcessError, TimeoutError, ImportError) as _:
            remove_mujoco_build()  # Cleans the installation.
    raise Exception("Failed to compile mujoco_py.")


def get_functions_from_cymj():
    cymj = load_cymj()
    functions = dict2()
    for func_name in dir(cymj):
        if func_name.startswith("_mj"):
            setattr(functions, func_name[1:], getattr(cymj, func_name))
    return functions