import distutils
import glob
import os
import shutil
import subprocess
import sys
import types
from distutils.core import Extension
from distutils.dist import Distribution
from distutils.sysconfig import customize_compiler
from os.path import abspath, dirname, exists, join, getmtime
from random import choice
from shutil import move
from string import ascii_lowercase
from importlib.machinery import ExtensionFileLoader

import numpy as np
from cffi import FFI
from Cython.Build import cythonize
from Cython.Distutils.old_build_ext import old_build_ext as build_ext

from mujoco_py.utils import discover_mujoco


def get_nvidia_version():
    cmd = 'nvidia-smi --query-gpu=driver_version --format=csv,noheader --id=0'
    try:
        return subprocess.check_output(cmd.split(), universal_newlines=True)
    except (FileNotFoundError, subprocess.CalledProcessError):
        return None


def get_nvidia_lib_dir():
    nvidia_version = get_nvidia_version()
    if nvidia_version is None:
        return None
    root = os.path.abspath(os.sep)
    major_version = nvidia_version.split('.')[0]
    for nvidia_lib_dir in [join(root, 'usr', 'local', 'nvidia', 'lib64'),
                           join(root, 'usr', 'lib', 'nvidia-' + major_version)]:
        if exists(nvidia_lib_dir):
            return nvidia_lib_dir


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
        if get_nvidia_lib_dir() is not None and os.getenv('MUJOCO_PY_FORCE_CPU') is None:
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
    return load_dynamic_ext('cymj', cext_so_path)


def load_dynamic_ext(name, path):
    ''' Load compiled shared object and return as python module. '''
    loader = ExtensionFileLoader(name, path)
    return loader.load_module()


def get_marker_file_path(obj):
    return '%s.meta' % obj


def should_compile(obj, src):
    if not os.path.isfile(obj):
        return True
    marker_file_path = get_marker_file_path(obj)
    if not os.path.isfile(marker_file_path):
        return True
    src_modification_time = '%f' % os.stat(src).st_mtime
    with open(marker_file_path, 'r') as marker_file:
        last_src_modification_time = marker_file.read()
        if last_src_modification_time == src_modification_time:
            return False
    return True


def save_marker(obj, src):
    marker_file_path = get_marker_file_path(obj)
    with open(marker_file_path, 'w') as marker_file:
        marker_file.write('%f' % os.stat(src).st_mtime)


def caching_compile(self, sources, output_dir=None, macros=None, include_dirs=None,
                    debug=0, extra_preargs=None, extra_postargs=None, depends=None):
    macros, objects, extra_postargs, pp_opts, build = self._setup_compile(
        output_dir, macros, include_dirs, sources, depends, extra_postargs)
    cc_args = self._get_cc_args(pp_opts, debug, extra_preargs)

    for obj in objects:
        try:
            src, ext = build[obj]
        except KeyError:
            continue
        if should_compile(obj, src):
            self._compile(obj, src, ext, cc_args, extra_postargs, pp_opts)
            save_marker(obj, src)
        else:
            print('%s up to date, skipping.' % obj)
    return objects
                

def enable_compiler_caching(compiler):
    ''' Patch 'compile' in UnixCCompiler in order to ignore existing artifacts. '''
    if compiler.compiler_type == 'unix':
        compiler.compile = types.MethodType(caching_compile, compiler) 


class custom_build_ext(build_ext):
    """
    Custom build_ext to suppress the "-Wstrict-prototypes" warning.
    It arises from the fact that we're using C++. This seems to be
    the cleanest way to get rid of the extra flag.

    See http://stackoverflow.com/a/36293331/248400
    """

    def build_extensions(self):
        customize_compiler(self.compiler)
        enable_compiler_caching(self.compiler)

        try:
            self.compiler.compiler_so.remove("-Wstrict-prototypes")
        except (AttributeError, ValueError):
            pass
        build_ext.build_extensions(self)


def fix_shared_library(so_file, name, library_path):
    ''' Used to fixup shared libraries on Linux '''
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


def manually_link_libraries(mjpro_path, raw_cext_dll_path):
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

    mj_bin_path = join(mjpro_path, 'bin')

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

    def build_base(self):
        return self.__class__.__name__

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
                                '_pyxbld_%s' % self.build_base())
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


class LinuxExtensionBuilder(MujocoExtensionBuilder):

    def __init__(self, mjpro_path):
        super().__init__(mjpro_path)

    def build_base(self):
        return LinuxExtensionBuilder.__name__


class LinuxCPUExtensionBuilder(LinuxExtensionBuilder):

    def __init__(self, mjpro_path):
        super().__init__(mjpro_path)

        self.extension.sources.append(
            join(self.CYMJ_DIR_PATH, "gl", "osmesashim.c"))
        self.extension.libraries.extend(['glewosmesa', 'OSMesa', 'GL'])
        self.extension.runtime_library_dirs = [join(mjpro_path, 'bin')]


class LinuxGPUExtensionBuilder(LinuxExtensionBuilder):

    def __init__(self, mjpro_path):
        super().__init__(mjpro_path)

        self.extension.sources.append(self.CYMJ_DIR_PATH + "/gl/eglshim.c")
        self.extension.include_dirs.append(self.CYMJ_DIR_PATH + '/vendor/egl')
        self.extension.libraries.extend(['glewegl'])
        self.extension.runtime_library_dirs = [join(mjpro_path, 'bin')]

    def _build_impl(self):
        so_file_path = super()._build_impl()
        fix_shared_library(so_file_path, 'libOpenGL.so',
                           join(get_nvidia_lib_dir(), 'libOpenGL.so.0'))
        fix_shared_library(so_file_path, 'libEGL.so',
                           join(get_nvidia_lib_dir(), 'libEGL.so.1'))
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
        return manually_link_libraries(self.mjpro_path, so_file_path)


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


def build_fn_cleanup(name):
    '''
    Cleanup files generated by building callback.
    Set the MUJOCO_PY_DEBUG_FN_BUILDER environment variable to disable cleanup.
    '''
    if not os.environ.get('MUJOCO_PY_DEBUG_FN_BUILDER', False):
        for f in glob.glob(name + '*'):
            try:
                os.remove(f)
            except PermissionError as e:
                # This happens trying to remove libraries on appveyor
                print('Error removing {}, continuing anyway: {}'.format(f, e))


def build_callback_fn(function_string, userdata_names=[]):
    '''
    Builds a C callback function and returns a function pointer int.

        function_string : str
            This is a string of the C function to be compiled
        userdata_names : list or tuple
            This is an optional list to defince convenience names

    We compile and link and load the function, and return a function pointer.
    See `MjSim.set_substep_callback()` for an example use of these callbacks.

    The callback function should match the signature:
        void fun(const mjModel *m, mjData *d);

    Here's an example function_string:
        ```
        """
        #include <stdio.h>
        void fun(const mjModel* m, mjData* d) {
            printf("hello");
        }
        """
        ```

    Input and output for the function pass through userdata in the data struct:
        ```
        """
        void fun(const mjModel* m, mjData* d) {
            d->userdata[0] += 1;
        }
        """
        ```

    `userdata_names` is expected to match the model where the callback is used.
    These can bet set on a model with:
        `model.set_userdata_names([...])`

    If `userdata_names` is supplied, convenience `#define`s are added for each.
    For example:
        `userdata_names = ['my_sum']`
    Will get gerenerated into the extra line:
        `#define my_sum d->userdata[0]`
    And prepended to the top of the function before compilation.
    Here's an example that takes advantage of this:
        ```
        """
        void fun(const mjModel* m, mjData* d) {
            for (int i = 0; i < m->nu; i++) {
                my_sum += d->ctrl[i];
            }
        }
        """
        ```
    Note these are just C `#define`s and are limited in how they can be used.

    After compilation, the built library containing the function is loaded
    into memory and all of the files (including the library) are deleted.
    To retain these for debugging set the `MUJOCO_PY_DEBUG_FN_BUILDER` envvar.

    To save time compiling, these function pointers may be re-used by many
    different consumers.  They are thread-safe and don't acquire the GIL.

    See the file `tests/test_substep.py` for additional examples,
    including an example which iterates over contacts to compute penetrations.
    '''
    assert isinstance(userdata_names, (list, tuple)), \
        'invalid userdata_names: {}'.format(userdata_names)
    ffibuilder = FFI()
    ffibuilder.cdef('extern uintptr_t __fun;')
    name = '_fn_' + ''.join(choice(ascii_lowercase) for _ in range(15))
    source_string = '#include <mujoco.h>\n'
    # Add defines for each userdata to make setting them easier
    for i, data_name in enumerate(userdata_names):
        source_string += '#define {} d->userdata[{}]\n'.format(data_name, i)
    source_string += function_string
    source_string += '\nuintptr_t __fun = (uintptr_t) fun;'
    # Link against mujoco so we can call mujoco functions from within callback
    ffibuilder.set_source(name, source_string,
                          include_dirs=[join(mjpro_path, 'include')],
                          library_dirs=[join(mjpro_path, 'bin')],
                          libraries=['mujoco150'])
    # Catch compilation exceptions so we can cleanup partial files in that case
    try:
        library_path = ffibuilder.compile(verbose=True)
    except Exception as e:
        build_fn_cleanup(name)
        raise e
    # On Mac the MuJoCo library is linked strangely, so we have to fix it here
    if sys.platform == 'darwin':
        fixed_library_path = manually_link_libraries(mjpro_path, library_path)
        move(fixed_library_path, library_path)  # Overwrite with fixed library
    module = load_dynamic_ext(name, library_path)
    # Now that the module is loaded into memory, we can actually delete it
    build_fn_cleanup(name)
    return module.lib.__fun


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
