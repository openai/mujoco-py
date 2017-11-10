import os
import time
from tempfile import mkdtemp
from unittest.mock import MagicMock
from mujoco_py.builder import enable_compiler_caching


def touch(file_path):
    with open(file_path, 'w') as touched_file:
        touched_file.write('foo')


def artifact(temp_dir, artifact_name):
    return os.path.join(temp_dir, artifact_name)


def mock_compiler_side_effect(obj, src, ext, cc_args, extra_postargs, pp_opts):
    touch(obj)


def mock_compiler(temp_dir, setup):
    compiler = MagicMock()
    compiler.compiler_type = 'unix'
    compiler._get_cc_args = MagicMock(return_value='')
    compiler._setup_compile = MagicMock(return_value=(
        None, setup.keys(), None, None, setup))
    compiler._compile = MagicMock(side_effect=mock_compiler_side_effect)
    return compiler


def test_caching_compiler():
    temp_dir = mkdtemp()
    
    a_o = artifact(temp_dir, 'a.o')
    a_c = artifact(temp_dir, 'a.c')
    a_o_meta = artifact(temp_dir, 'a.o.meta')
    touch(a_c)
    
    b_o = artifact(temp_dir, 'b.o')
    b_cc = artifact(temp_dir, 'b.cc')
    b_o_meta = artifact(temp_dir, 'b.o.meta')
    touch(b_cc)
    
    compiler = mock_compiler(temp_dir, {a_o : (a_c, 'c'), b_o : (b_cc, 'cc')})
    enable_compiler_caching(compiler)

    compiler.compile(None)
    assert compiler._compile.call_count is 2

    compiler.compile(None)
    assert compiler._compile.call_count is 2

    os.remove(a_o)
    os.remove(b_o_meta)
    compiler.compile(None)
    assert compiler._compile.call_count is 4

    time.sleep(0.1)
    touch(a_c)
    compiler.compile(None)
    assert compiler._compile.call_count is 5
