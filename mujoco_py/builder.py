import distutils
import imp
import os
import shutil
import subprocess
import sys
from distutils.core import Extension
from distutils.dist import Distribution
from distutils.sysconfig import customize_compiler
from os.path import abspath, dirname, exists, join, getmtime
from shutil import move

import numpy as np
from Cython.Build import cythonize
from Cython.Distutils.old_build_ext import old_build_ext as build_ext

from mujoco_py.utils import discover_mujoco


def load_cython_ext(mjpro_path):
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

    if sys.platform == 'darwin':
        Builder = MacExtensionBuilder
    elif sys.platform == 'linux':
        if exists('/usr/local/nvidia/lib64'):
            Builder = LinuxGPUExtensionBuilder
        else:
            Builder = LinuxCPUExtensionBuilder
    elif sys.platform.startswith("win"):
        Builder = WindowsExtensionBuilder
    else:
        raise RuntimeError("Unsupported platform %s" % sys.platform)

    builder = Builder(mjpro_path)
    cext_so_path = builder.get_so_file_path()
    if not exists(cext_so_path):
        cext_so_path = builder.build()
    mod = imp.load_dynamic("cymj", cext_so_path)
    return mod


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
    ldd_output = subprocess.check_output(
        ['ldd', so_file]).decode('utf-8')

    if name in ldd_output:
        subprocess.check_call(['patchelf',
                               '--remove-needed',
                               name,
                               so_file])
    subprocess.check_call(
        ['patchelf', '--add-needed',
         library_path,
         so_file])


class MujocoExtensionBuilder():

    CYMJ_DIR_PATH = abspath(dirname(__file__))

    def __init__(self, mjpro_path):
        self.mjpro_path = mjpro_path
        self.extension = Extension(
            'mujoco_py.cymj',
            sources=[join(self.CYMJ_DIR_PATH, "cymj.pyx")],
            include_dirs=[
                self.CYMJ_DIR_PATH,
                join(mjpro_path, 'include'),
                np.get_include(),
            ],
            libraries=['mujoco150'],
            library_dirs=[join(mjpro_path, 'bin')],
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
                                '_pyxbld_%s' % self.__class__.__name__)
        dist.parse_command_line()
        obj_build_ext = dist.get_command_obj("build_ext")
        dist.run_commands()
        built_so_file_path, = obj_build_ext.get_outputs()
        return built_so_file_path

    def get_so_file_path(self):
        dir_path = abspath(dirname(__file__))
        return join(dir_path, "generated", "cymj_%s.so" % (
            self.__class__.__name__.lower()))


class WindowsExtensionBuilder(MujocoExtensionBuilder):

    def __init__(self, mjpro_path):
        super().__init__(mjpro_path)
        os.environ["PATH"] += ";" + join(mjpro_path, "bin")
        self.extension.sources.append(self.CYMJ_DIR_PATH + "/gl/dummyshim.c")


class LinuxCPUExtensionBuilder(MujocoExtensionBuilder):

    def __init__(self, mjpro_path):
        super().__init__(mjpro_path)

        self.extension.sources.append(
            join(self.CYMJ_DIR_PATH, "gl", "osmesashim.c"))
        self.extension.libraries.extend(['glewosmesa', 'OSMesa'])
        self.extension.runtime_library_dirs = [join(mjpro_path, 'bin')]


class LinuxGPUExtensionBuilder(MujocoExtensionBuilder):

    def __init__(self, mjpro_path):
        super().__init__(mjpro_path)

        self.extension.sources.append(self.CYMJ_DIR_PATH + "/gl/eglshim.c")
        self.extension.include_dirs.append(self.CYMJ_DIR_PATH + '/vendor/egl')
        self.extension.libraries.extend(['glewegl'])
        self.extension.runtime_library_dirs = [join(mjpro_path, 'bin')]

    def _build_impl(self):
        so_file_path = super()._build_impl()
        nvidia_lib_dir = '/usr/local/nvidia/lib64/'
        fix_shared_library(so_file_path, 'libOpenGL.so',
                           join(nvidia_lib_dir, 'libOpenGL.so.0'))
        fix_shared_library(so_file_path, 'libEGL.so',
                           join(nvidia_lib_dir, 'libEGL.so.1'))
        return so_file_path


class MacExtensionBuilder(MujocoExtensionBuilder):

    def __init__(self, mjpro_path):
        super().__init__(mjpro_path)

        self.extension.sources.append(self.CYMJ_DIR_PATH + "/gl/dummyshim.c")
        self.extension.libraries.extend(['glfw.3'])
        self.extension.define_macros = [('ONMAC', None)]
        self.extension.runtime_library_dirs = [join(mjpro_path, 'bin')]

    def _build_impl(self):
        # Prefer GCC 6 for now since GCC 7 may behave differently.
        c_compilers = ['/usr/local/bin/gcc-6', '/usr/local/bin/gcc-7']
        available_c_compiler = None
        for c_compiler in c_compilers:
            if distutils.spawn.find_executable(c_compiler) is not None:
                available_c_compiler = c_compiler
                break
        if available_c_compiler is None:
            raise RuntimeError(
                'Could not find GCC 6 or GCC 7 executable.\n\n'
                'HINT: On OS X, install GCC 6 with '
                '`brew install gcc --without-multilib`.')
        os.environ['CC'] = available_c_compiler

        so_file_path = super()._build_impl()
        del os.environ['CC']
        return self.manually_link_libraries(so_file_path)

    def manually_link_libraries(self, raw_cext_dll_path):
        root, ext = os.path.splitext(raw_cext_dll_path)
        final_cext_dll_path = root + '_final' + ext

        # If someone else already built the final DLL, don't bother
        # recreating it here, even though this should still be idempotent.
        if (exists(final_cext_dll_path) and
                getmtime(final_cext_dll_path) >= getmtime(raw_cext_dll_path)):
            return final_cext_dll_path

        tmp_final_cext_dll_path = final_cext_dll_path + '~'
        shutil.copyfile(raw_cext_dll_path, tmp_final_cext_dll_path)

        mj_bin_path = join(self.mjpro_path, 'bin')

        # Fix the rpath of the generated library -- i lost the Stackoverflow
        # reference here
        from_mujoco_path = '@executable_path/libmujoco150.dylib'
        to_mujoco_path = '%s/libmujoco150.dylib' % mj_bin_path
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


mjpro_path, key_path = discover_mujoco()
cymj = load_cython_ext(mjpro_path)


# Trick to expose all mj* functions from mujoco in mujoco_py.*
class dict2(object):
    pass


functions = dict2()
for func_name in dir(cymj):
    if func_name.startswith("_mj"):
        setattr(functions, func_name[1:], getattr(cymj, func_name))

functions.mj_activate(key_path)

# Set user-defined callbacks that raise assertion with message
cymj.set_warning_callback(user_warning_raise_exception)
