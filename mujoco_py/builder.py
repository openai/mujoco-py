import distutils
import os
import shutil
import sys
from distutils.core import Extension
from distutils.dist import Distribution
from distutils.sysconfig import customize_compiler
from os.path import abspath, dirname, exists, join, getmtime
from random import choice
from shutil import move
from string import ascii_lowercase
from importlib.machinery import ExtensionFileLoader
import glob

import numpy as np
from cffi import FFI
from Cython.Build import cythonize
from Cython.Distutils.old_build_ext import old_build_ext as build_ext
from mujoco_py.version import get_version
from lockfile import LockFile
import subprocess

from mujoco_py.utils import discover_mujoco, MISSING_KEY_MESSAGE


def get_nvidia_lib_dir():
    exists_nvidia_smi = subprocess.call("type nvidia-smi", shell=True,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE) == 0
    if not exists_nvidia_smi:
        return None
    docker_path = '/usr/local/nvidia/lib64'
    if exists(docker_path):
        return docker_path
    paths = glob.glob('/usr/lib/nvidia-[0-9][0-9][0-9]')
    paths = sorted(paths)
    if len(paths) == 0:
        return None
    if len(paths) > 1:
        print("Choosing the latest nvidia driver: %s, among %s" % (paths[-1], str(paths)))

    return paths[-1]


def load_cython_ext(mujoco_path):
    """
    Loads the cymj Cython extension. This is safe to be called from
    multiple processes running on the same machine.

    Cython only gives us back the raw path, regardless of whether
    it found a cached version or actually compiled. Since we do
    non-idempotent postprocessing of the DLL, be extra careful
    to only do that once and then atomically move to the final
    location.
    """
    if ('glfw' in sys.modules and
            'mujoco' in abspath(sys.modules["glfw"].__file__)):
        print('''
WARNING: Existing glfw python module detected!

MuJoCo comes with its own version of GLFW, so it's preferable to use that one.

The easy solution is to `import mujoco_py` _before_ `import glfw`.
''')

    lib_path = os.path.join(mujoco_path, "bin")
    if sys.platform == 'darwin':
        Builder = MacExtensionBuilder
    elif sys.platform == 'linux':
        _ensure_set_env_var("LD_LIBRARY_PATH", lib_path)
        if os.getenv('MUJOCO_PY_FORCE_CPU') is None and get_nvidia_lib_dir() is not None:
            _ensure_set_env_var("LD_LIBRARY_PATH", get_nvidia_lib_dir())
            Builder = LinuxGPUExtensionBuilder
        else:
            Builder = LinuxCPUExtensionBuilder
    else:
        raise RuntimeError("Unsupported platform %s" % sys.platform)

    builder = Builder(mujoco_path)
    cext_so_path = builder.get_so_file_path()

    lockpath = os.path.join(os.path.dirname(cext_so_path), 'mujocopy-buildlock')

    with LockFile(lockpath):
        mod = None
        force_rebuild = os.environ.get('MUJOCO_PY_FORCE_REBUILD')
        if force_rebuild:
            # Try to remove the old file, ignore errors if it doesn't exist
            print("Removing old mujoco_py cext", cext_so_path)
            try:
                os.remove(cext_so_path)
            except OSError:
                pass
        if exists(cext_so_path):
            try:
                mod = load_dynamic_ext('cymj', cext_so_path)
            except ImportError:
                print("Import error. Trying to rebuild mujoco_py.")
        if mod is None:
            cext_so_path = builder.build()
            mod = load_dynamic_ext('cymj', cext_so_path)
    return mod


def _ensure_set_env_var(var_name, lib_path):
    paths = os.environ.get(var_name, "").split(":")
    paths = [os.path.abspath(path) for path in paths]
    if lib_path not in paths:
        raise Exception("\nMissing path to your environment variable. \n"
                        "Current values %s=%s\n"
                        "Please add following line to .bashrc:\n"
                        "export %s=$%s:%s" % (var_name, os.environ.get(var_name, ""),
                                              var_name, var_name, lib_path))


def load_dynamic_ext(name, path):
    ''' Load compiled shared object and return as python module. '''
    loader = ExtensionFileLoader(name, path)
    return loader.load_module()


class custom_build_ext(build_ext):
    """
    Custom build_ext to suppress the "-Wstrict-prototypes" warning.
    It arises from the fact that we're using C++. This seems to be
    the cleanest way to get rid of the extra flag.

    See http://stackoverflow.com/a/36293331/248400
    """

    def build_extensions(self):
        customize_compiler(self.compiler)

        try:
            self.compiler.compiler_so.remove("-Wstrict-prototypes")
        except (AttributeError, ValueError):
            pass
        build_ext.build_extensions(self)


def fix_shared_library(so_file, name, library_path):
    ''' Used to fixup shared libraries on Linux '''
    subprocess.check_call(['patchelf', '--remove-rpath', so_file])
    ldd_output = subprocess.check_output(['ldd', so_file]).decode('utf-8')

    if name in ldd_output:
        subprocess.check_call(['patchelf', '--remove-needed', name, so_file])
    subprocess.check_call(['patchelf', '--add-needed', library_path, so_file])


def manually_link_libraries(mujoco_path, raw_cext_dll_path):
    ''' Used to fix mujoco library linking on Mac '''
    root, ext = os.path.splitext(raw_cext_dll_path)
    final_cext_dll_path = root + '_final' + ext

    # If someone else already built the final DLL, don't bother
    # recreating it here, even though this should still be idempotent.
    if (exists(final_cext_dll_path) and
            getmtime(final_cext_dll_path) >= getmtime(raw_cext_dll_path)):
        return final_cext_dll_path

    tmp_final_cext_dll_path = final_cext_dll_path + '~'
    shutil.copyfile(raw_cext_dll_path, tmp_final_cext_dll_path)

    mj_bin_path = join(mujoco_path, 'bin')

    # Fix the rpath of the generated library -- i lost the Stackoverflow
    # reference here
    from_mujoco_path = '@executable_path/libmujoco200.dylib'
    to_mujoco_path = '%s/libmujoco200.dylib' % mj_bin_path
    subprocess.check_call(['install_name_tool',
                           '-change',
                           from_mujoco_path,
                           to_mujoco_path,
                           tmp_final_cext_dll_path])

    from_glfw_path = 'libglfw.3.dylib'
    to_glfw_path = os.path.join(mj_bin_path, 'libglfw.3.dylib')
    subprocess.check_call(['install_name_tool',
                           '-change',
                           from_glfw_path,
                           to_glfw_path,
                           tmp_final_cext_dll_path])

    os.rename(tmp_final_cext_dll_path, final_cext_dll_path)
    return final_cext_dll_path


class MujocoExtensionBuilder():

    CYMJ_DIR_PATH = abspath(dirname(__file__))

    def __init__(self, mujoco_path):
        self.mujoco_path = mujoco_path
        python_version = str(sys.version_info.major) + str(sys.version_info.minor)
        self.version = '%s_%s_%s' % (get_version(), python_version, self.build_base())
        self.extension = Extension(
            'mujoco_py.cymj',
            sources=[join(self.CYMJ_DIR_PATH, "cymj.pyx")],
            include_dirs=[
                self.CYMJ_DIR_PATH,
                join(mujoco_path, 'include'),
                np.get_include(),
            ],
            libraries=['mujoco200'],
            library_dirs=[join(mujoco_path, 'bin')],
            extra_compile_args=[
                '-fopenmp',  # needed for OpenMP
                '-w',  # suppress numpy compilation warnings
            ],
            extra_link_args=['-fopenmp'],
            language='c')

    def build(self):
        built_so_file_path = self._build_impl()
        new_so_file_path = self.get_so_file_path()
        move(built_so_file_path, new_so_file_path)
        return new_so_file_path

    def build_base(self):
        return self.__class__.__name__.lower()

    def _build_impl(self):
        dist = Distribution({
            "script_name": None,
            "script_args": ["build_ext"]
        })
        dist.ext_modules = cythonize([self.extension])
        dist.include_dirs = []
        dist.cmdclass = {'build_ext': custom_build_ext}
        build = dist.get_command_obj('build')
        # following the convention of cython's pyxbuild and naming
        # base directory "_pyxbld"
        build.build_base = join(self.CYMJ_DIR_PATH, 'generated',
                                '_pyxbld_%s' % (self.version))
        dist.parse_command_line()
        obj_build_ext = dist.get_command_obj("build_ext")
        dist.run_commands()
        built_so_file_path, = obj_build_ext.get_outputs()
        return built_so_file_path

    def get_so_file_path(self):
        dir_path = abspath(dirname(__file__))
        python_version = str(sys.version_info.major) + str(sys.version_info.minor)
        return join(dir_path, "generated", "cymj_{}_{}.so".format(self.version, python_version))


class LinuxCPUExtensionBuilder(MujocoExtensionBuilder):

    def __init__(self, mujoco_path):
        super().__init__(mujoco_path)

        self.extension.sources.append(
            join(self.CYMJ_DIR_PATH, "gl", "osmesashim.c"))
        self.extension.libraries.extend(['glewosmesa', 'OSMesa', 'GL'])
        self.extension.runtime_library_dirs = [join(mujoco_path, 'bin')]

    def _build_impl(self):
        so_file_path = super()._build_impl()
        # Removes absolute paths to libraries. Allows for dynamic loading.
        fix_shared_library(so_file_path, 'libmujoco200.so', 'libmujoco200.so')
        fix_shared_library(so_file_path, 'libglewosmesa.so', 'libglewosmesa.so')
        return so_file_path


class LinuxGPUExtensionBuilder(MujocoExtensionBuilder):

    def __init__(self, mujoco_path):
        super().__init__(mujoco_path)

        self.extension.sources.append(self.CYMJ_DIR_PATH + "/gl/eglshim.c")
        self.extension.include_dirs.append(self.CYMJ_DIR_PATH + '/vendor/egl')
        self.extension.libraries.extend(['glewegl'])
        self.extension.runtime_library_dirs = [join(mujoco_path, 'bin')]

    def _build_impl(self):
        so_file_path = super()._build_impl()
        fix_shared_library(so_file_path, 'libOpenGL.so', 'libOpenGL.so.0')
        fix_shared_library(so_file_path, 'libEGL.so', 'libEGL.so.1')
        fix_shared_library(so_file_path, 'libmujoco200.so', 'libmujoco200.so')
        fix_shared_library(so_file_path, 'libglewegl.so', 'libglewegl.so')
        return so_file_path


class MacExtensionBuilder(MujocoExtensionBuilder):

    def __init__(self, mujoco_path):
        super().__init__(mujoco_path)

        self.extension.sources.append(self.CYMJ_DIR_PATH + "/gl/dummyshim.c")
        self.extension.libraries.extend(['glfw.3'])
        self.extension.define_macros = [('ONMAC', None)]
        self.extension.runtime_library_dirs = [join(mujoco_path, 'bin')]

    def _build_impl(self):
        if not os.environ.get('CC'):
            # Known-working versions of GCC on mac
            c_compilers = ['/usr/local/bin/gcc-6',
                           '/usr/local/bin/gcc-7',
                           '/usr/local/bin/gcc-8']
            available_c_compiler = None
            for c_compiler in c_compilers:
                if distutils.spawn.find_executable(c_compiler) is not None:
                    available_c_compiler = c_compiler
                    break
            if available_c_compiler is None:
                raise RuntimeError(
                    'Could not find GCC executable.\n\n'
                    'HINT: On OS X, install GCC with '
                    '`brew install gcc`.')
            os.environ['CC'] = available_c_compiler

            so_file_path = super()._build_impl()
            del os.environ['CC']
        else:  # User-directed c compiler
            so_file_path = super()._build_impl()
        return manually_link_libraries(self.mujoco_path, so_file_path)


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


class ignore_mujoco_warnings:
    """
    Class to turn off mujoco warning exceptions within a scope. Useful for
    large, vectorized rollouts.
    """

    def __enter__(self):
        self.prev_user_warning = cymj.get_warning_callback()
        cymj.set_warning_callback(user_warning_ignore_exception)
        return self

    def __exit__(self, type, value, traceback):
        cymj.set_warning_callback(self.prev_user_warning)


def find_key():
    ''' Try to find the key file, if missing, print out a big message '''
    if exists(key_path):
        return
    print(MISSING_KEY_MESSAGE.format(key_path), file=sys.stderr)


def activate():
    functions.mj_activate(key_path)


mujoco_path, key_path = discover_mujoco()
cymj = load_cython_ext(mujoco_path)


# Trick to expose all mj* functions from mujoco in mujoco_py.*
class dict2(object):
    pass


functions = dict2()
for func_name in dir(cymj):
    if func_name.startswith("_mj"):
        setattr(functions, func_name[1:], getattr(cymj, func_name))

# Set user-defined callbacks that raise assertion with message
cymj.set_warning_callback(user_warning_raise_exception)
