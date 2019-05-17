# This is a stand alone file!!
# Don't not introduce any dependency on mujoco_py.

import sys
import distutils
import os
import shutil
from os.path import abspath, dirname, exists, join, getmtime
from shutil import move
import glob

import numpy as np
from Cython.Build import cythonize
from Cython.Distutils.old_build_ext import old_build_ext as build_ext
from lockfile import LockFile, LockTimeout
import subprocess

# Ensures that mujoco_py is not imported.
sys.modules['mujoco_py'] = None


def load_dynamic_ext():
    raise Exception("load_dynamic_ext should have been "
                    "overridden by discover_mujoco from utils.py")


def discover_mujoco():
    raise Exception("discover_mujoco function should have been "
                    "overridden by discover_mujoco from utils.py")


# This way we import functions from version.py, and utils.py without
# relying on mujoco_py package.
with open(join(dirname(__file__), "version.py")) as version_file:
    exec(version_file.read())


with open(join(dirname(__file__), "utils.py")) as version_file:
    exec(version_file.read())

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


def remove_mujoco_build():
    # Removes previously compiled mujoco_py files.
    print("Removing previously compiled mujoco_py files.")
    path = os.path.join(os.path.dirname(__file__), "generated")
    for fname in glob.glob(f"{path}/*.so"):
        os.remove(fname)
    for dirname in glob.glob(f"{path}/_pyxbld*"):
        shutil.rmtree(dirname, ignore_errors=True)
    shutil.rmtree(f"{path}/__pycache__", ignore_errors=True)


def _ensure_set_env_var(var_name, lib_path):
    paths = os.environ.get(var_name, "").split(":")
    paths = [os.path.abspath(path) for path in paths]
    if lib_path not in paths:
        raise Exception("\nMissing path to your environment variable. \n"
                        "Current values %s=%s\n"
                        "Please add following line to .bashrc:\n"
                        "export %s=$%s:%s" % (var_name, os.environ.get(var_name, ""),
                                              var_name, var_name, lib_path))


class custom_build_ext(build_ext):
    """
    Custom build_ext to suppress the "-Wstrict-prototypes" warning.
    It arises from the fact that we're using C++. This seems to be
    the cleanest way to get rid of the extra flag.

    See http://stackoverflow.com/a/36293331/248400
    """

    def build_extensions(self):
        if "customize_compiler" in globals():
            del globals()["customize_compiler"]
        from distutils.sysconfig import customize_compiler
        customize_compiler(self.compiler)
        try:
            self.compiler.compiler_so.remove("-Wstrict-prototypes")
        except (AttributeError, ValueError):
            pass
        self.force = True
        build_ext.build_extensions(self)


def fix_shared_library(so_file, name, library_path):
    ''' Used to fixup shared libraries on Linux '''
    subprocess.check_call(['patchelf', '--remove-rpath', so_file])
    ldd_output = subprocess.check_output(['ldd', so_file]).decode('utf-8')

    if name in ldd_output:
        subprocess.check_call(['patchelf', '--remove-needed', name, so_file])
    subprocess.check_call(['patchelf', '--add-needed', library_path, so_file])


class MujocoExtensionBuilder():

    CYMJ_DIR_PATH = abspath(dirname(__file__))

    def __init__(self, mujoco_path):
        self.mujoco_path = mujoco_path
        python_version = str(sys.version_info.major) + str(sys.version_info.minor)
        self.version = '%s_%s_%s' % (get_version(), python_version, self.build_base())
        if "Extension" in globals():
            del globals()["Extension"]
        from distutils.core import Extension
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
        if "Distribution" in globals():
            del globals()["Distribution"]
        from distutils.dist import Distribution
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


def main():
    """
    Ensures that cymj Cython extension is compiled. This is safe to be called from
    multiple processes running on the same machine.

    Cython only gives us back the raw path, regardless of whether
    it found a cached version or actually compiled. Since we do
    non-idempotent postprocessing of the .so, be extra careful
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
    mujoco_path, key_path = discover_mujoco()
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

    lockpath = os.path.join(os.path.dirname(__file__),
                            "generated",
                            "mujocopy-buildlock")
    lock = LockFile(lockpath)
    try:
        print("Trying to lock for mujoco_py compilation.")
        # XXXX 80 -> 150
        lock.acquire(timeout=80)
    except LockTimeout:
        # Processed that has acquired lock might be dead
        # (e.g. due to being interrupted during execution).
        # Therefore, after timeout we should move forward with compilation anyway.
        print("\nAcquiring lock despite of it being taken. "
              "Timeout has occurred.\n")
        lock.break_lock()
        exit(-1)
    try:
        builder = Builder(mujoco_path)
        cext_so_path = builder.get_so_file_path()
        mod = None
        force_rebuild = os.environ.get('MUJOCO_PY_FORCE_REBUILD')
        if force_rebuild:
            # Try to remove the old file, ignore errors if it doesn't exist
            remove_mujoco_build()
        if exists(cext_so_path):
            try:
                load_dynamic_ext('cymj', cext_so_path)
            except ImportError:
                print("Import error. Trying to rebuild mujoco_py.")
        if mod is None:
            remove_mujoco_build()  # ensures that compile environment is clean.
            print("Compiling mujoco_py. Might take several minutes.")
            builder = Builder(mujoco_path)
            cext_so_path = builder.build()
            load_dynamic_ext('cymj', cext_so_path)
    finally:
        lock.release()
    exit(0)

if __name__ == "__main__":
    main()
