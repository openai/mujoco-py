#!/usr/bin/env python

import codecs
import os
import pycparser
import re
import subprocess
import sys
import tempfile
import traceback


from collections import OrderedDict
from pycparser.c_ast import ArrayDecl, TypeDecl, PtrDecl, Union, BinaryOp, UnaryOp


c_compiler = "cc"
if sys.platform.startswith("win"):
    c_compiler = "cl"


def tryint(x):
    try:
        return int(x)
    except:
        return x


def get_struct_dict(struct, struct_name, array_shapes):
    struct_dict = OrderedDict()
    struct_dict[struct_name] = OrderedDict([('scalars', []),
                                            ('arrays', []),
                                            ('arrays2d', []),
                                            ('ptrs', []),
                                            ('depends_on_model', False)])
    for child in struct.children():
        child_name = child[1].name
        child_type = child[1].type
        decl = child[1].children()[0][1]
        if isinstance(child_type, ArrayDecl):
            if hasattr(decl.type.type, "names"):
                array_type = ' '.join(decl.type.type.names)
                array_size = extract_size_info(decl.dim)
                struct_dict[struct_name]['arrays'].append((child_name,
                                                           array_type,
                                                           array_size))
            elif isinstance(decl.type, PtrDecl):
                print("skipping pointer array: %s.%s" % (struct_name, child_name))
                continue
            elif hasattr(decl.type.type.type, "names"):
                # assuming a 2d array
                array_type = ' '.join(decl.type.type.type.names)
                s1 = extract_size_info(decl.dim)
                s2 = extract_size_info(decl.type.dim)
                struct_dict[struct_name]['arrays2d'].append((child_name, array_type, (s1, s2)))
            else:
                print("skipping unknown array case: %s.%s\n%s" % (struct_name, child_name, child))

        elif isinstance(child_type, TypeDecl):
            if isinstance(decl.type, pycparser.c_ast.Struct):
                fixed_name = decl.declname
                if fixed_name == "global":
                    fixed_name = "global_"
                name = struct_name + "_" + fixed_name
                child_struct_dict = get_struct_dict(
                    decl.type, name, array_shapes)
                struct_dict = OrderedDict(struct_dict, **child_struct_dict)
                struct_dict[struct_name]['scalars'].append((fixed_name, name))
            else:
                field_type = ' '.join(decl.type.names)
                struct_dict[struct_name]['scalars'].append(
                    (child_name, field_type))

        elif isinstance(child_type, PtrDecl):
            ptr_type = ' '.join(decl.type.type.names)
            n = struct_name + '.' + child_name
            if n not in array_shapes:
                print('Warning: skipping {} due to unknown shape'.format(n))
            else:
                struct_dict[struct_name]['ptrs'].append(
                    (child_name, ptr_type, array_shapes[n]))
                # Structs needing array shapes must get them through mjModel
                # but mjModel itself doesn't need to be passed an extra mjModel.
                # TODO: depends_on_model should be set to True if any member of this struct depends on mjModel
                # but currently that never happens.
                if struct_name != 'mjModel':
                    struct_dict[struct_name]['depends_on_model'] = True
        elif isinstance(child_type, Union):
            # I'm ignoring unions for now until we think they're necessary
            continue
        else:
            raise NotImplementedError

    assert isinstance(struct_dict, OrderedDict), 'Must be deterministic'
    return struct_dict


def extract_size_info(node):
    """
    Try to extract what integer value (or named reference) `node` contains.
    Can handle

        pycparser.c_ast.Constant
        pycparser.c_ast.BinaryOp(op="*", left, right)
        pycparser.c_ast.ID

    as long as `left` and `right` are either `Constant` or BinaryOp's that ultimately evaluate to constants.

    :param node: The AST node representing an integer constant.
    :return: The value
    """
    if isinstance(node, pycparser.c_ast.ID):
        return node.name
    elif isinstance(node, BinaryOp):
        if node.op == "*":
            return extract_size_info(node.left) * extract_size_info(node.right)
    elif isinstance(node, pycparser.c_ast.Constant):
        return int(node.value)
    raise NotImplementedError(str(node))


def format_size_argument(model_var_name, shape_def):
  if isinstance(shape_def, str):
    if shape_def.startswith('n'):
      return f'{model_var_name}.{shape_def}'
    m = re.match(r'(\d+)\s*\*\s*(n[a-zA-Z]*)', shape_def)
    if m:
      return f'{m.group(1)}*{model_var_name}.{m.group(2)}'

  return shape_def


def get_full_scr_lines(HEADER_DIR, HEADER_FILES):
    # ===== Read all header files =====
    file_contents = []
    for filename in HEADER_FILES:
        # mujoco 2.0 header files fail when parsed as utf-8
        with codecs.open(os.path.join(HEADER_DIR, filename), 'r', encoding='latin-1') as f:
            file_contents.append(f.read())
    full_src_lines = [line.strip()
                      for line in '\n'.join(file_contents).splitlines()]
    return full_src_lines


def get_array_shapes(full_src_lines):
    #  ===== Parse array shape hints =====
    array_shapes = OrderedDict()
    curr_struct_name = None
    for line in full_src_lines:
        # Current struct name
        m = re.match(r'struct (\w+)', line)
        if m:
            curr_struct_name = m.group(1)
            continue
        # Pointer with a shape comment
        m = re.match(r'\s*\w+\s*\*\s+(\w+);\s*//.*\((.+) x (.+)\)$', line)
        if m:
            name = curr_struct_name[1:] + '.' + m.group(1)
            assert name not in array_shapes
            array_shapes[name] = (tryint(m.group(2)), tryint(m.group(3)))
    return array_shapes


def get_processed_src(HEADER_DIR, full_src_lines):
    # ===== Preprocess header files =====
    with tempfile.NamedTemporaryFile(suffix='.h', delete=False) as f:
        f.write('\n'.join(full_src_lines).encode())
        f.flush()
        print("Saved all header information to: %s" % f.name)
        # -E: run preprocessor only
        # -P: don't generate debug lines starting with #
        # -I: include directory
        processed_src = subprocess.check_output(
            [c_compiler, '-E', '-P', '-I', HEADER_DIR, f.name]).decode()
    return processed_src


def get_full_struct_dict(processed_src, array_shapes):
    # ===== Parse and extract structs =====
    ast = pycparser.c_parser.CParser().parse(processed_src)
    struct_dict = OrderedDict()
    for node in ast.children():
        assert (node[1].name is None) == isinstance(
            node[1].type, pycparser.c_ast.Struct)
        if isinstance(node[1].type, pycparser.c_ast.Struct):
            (_, struct), = node[1].children()
            assert struct.name.startswith('_mj')
            struct_name = struct.name[1:]  # take out leading underscore
            assert struct_name not in struct_dict
            struct_dict = struct_dict.copy()
            struct_dict.update(get_struct_dict(struct, struct_name, array_shapes))
    assert isinstance(struct_dict, OrderedDict), 'Must be deterministic'
    return struct_dict


def get_const_from_enum(processed_src):
    # ===== Parse and extract structs =====
    ast = pycparser.c_parser.CParser().parse(processed_src)

    lines = []
    for node in ast.children():
        assert (node[1].name is None) == isinstance(
            node[1].type, pycparser.c_ast.Struct)
        struct = node[1].children()[0][1]

        # Check if it is an enum
        if hasattr(struct, "type") and isinstance(struct.type, pycparser.c_ast.Enum):
            lines.append(" # " + struct.type.name)

            # enum list os a list of key-value enumerations
            enumlist = struct.children()[0][1].children()[0][1].children()

            last_value = None

            for _, enum in enumlist:
                var = enum.name[2:]

                # Enum has two parts - name and value
                if enum.value is not None:
                    if isinstance(enum.value, BinaryOp):
                        # An enum is actually a binary operation
                        if enum.value.op == '<<':
                            # Parse and evaluate simple constant expression. Will throw if it's anything more complex
                            value = int(enum.value.children()[0][1].value) << int(enum.value.children()[1][1].value)
                        else:
                            raise NotImplementedError
                    elif isinstance(enum.value, UnaryOp):
                        # If we want to be correct we need to do a bit of parsing here....
                        if enum.value.op == '-':
                            # Again, if some assumptions I'm making here are not correct, this should throw
                            value = -int(enum.value.expr.value)
                        else:
                            raise NotImplementedError
                    else:
                        children = enum.value.children()

                        if len(children) > 0:
                            value = children[1][1].value
                        else:
                            value = enum.value.value

                        value = int(value)

                    last_value = value
                    new_line = str(var) + " = " + str(value)
                    lines.append(new_line)
                else:
                    assert(last_value is not None)
                    last_value += 1
                    lines.append(str(var) + " = " + str(last_value))
            lines.append("")
    return lines


def get_struct_wrapper(struct_dict):
    #  ===== Generate code =====
    structname2wrappername = OrderedDict()
    structname2wrapfuncname = OrderedDict()
    for name in struct_dict:
        assert name.startswith('mj')
        structname2wrappername[name] = 'PyMj' + name[2:]
        structname2wrapfuncname[name] = 'WrapMj' + name[2:]
    return structname2wrappername, structname2wrapfuncname


def _add_named_access_methods(obj_type, attr_name, attr_name_short):
    getter_name = obj_type
    if attr_name_short is not None:
        getter_name += "_" + attr_name_short
    reshape_suffix = ".reshape((3, 3))" if attr_name.endswith('mat') else ''
    code = """
    def get_{getter_name}(self, name):
        id = self._model.{obj_type}_name2id(name)
        return self._{attr_name}[id]{reshape_suffix}\n""".format(
        obj_type=obj_type, getter_name=getter_name, attr_name=attr_name, reshape_suffix=reshape_suffix)
    if getter_name != attr_name:
        code += """
    def get_{attr_name}(self, name):
        raise RuntimeError("get_{getter_name} should be used instead of get_{attr_name}")\n""".format(
            getter_name=getter_name, attr_name=attr_name)
    return code


def _add_named_jacobian_methods(obj_type):
    cap_type = obj_type.title()  # Capitalized
    code = """
    def get_{obj_type}_jacp(self, name, np.ndarray[double, ndim=1, mode="c"] jacp = None):
        id = self._model.{obj_type}_name2id(name)
        if jacp is None:
            jacp = np.zeros(3 * self._model.nv)
        cdef double * jacp_view = &jacp[0]
        mj_jac{cap_type}(self._model.ptr, self.ptr, jacp_view, NULL, id)
        return jacp

    def get_{obj_type}_jacr(self, name, np.ndarray[double, ndim=1, mode="c"] jacr = None):
        id = self._model.{obj_type}_name2id(name)
        if jacr is None:
            jacr = np.zeros(3 * self._model.nv)
        cdef double * jacr_view = &jacr[0]
        mj_jac{cap_type}(self._model.ptr, self.ptr, NULL, jacr_view, id)
        return jacr

    def get_{obj_type}_xvelp(self, name):
        id = self._model.{obj_type}_name2id(name)
        jacp = self.get_{obj_type}_jacp(name).reshape((3, self._model.nv))
        xvelp = np.dot(jacp, self.qvel)
        return xvelp

    def get_{obj_type}_xvelr(self, name):
        id = self._model.{obj_type}_name2id(name)
        jacr = self.get_{obj_type}_jacr(name).reshape((3, self._model.nv))
        xvelr = np.dot(jacr, self.qvel)
        return xvelr\n""".format(obj_type=obj_type, cap_type=cap_type)
    return code


def _add_jacobian_getters(obj_type):
    cap_type = obj_type.title()   # Capitalized
    code = '''
    @property
    def {obj_type}_jacp(self):
        jacps = np.zeros((self._model.n{obj_type}, 3 * self._model.nv))
        cdef double [:] jacp_view
        for i, jacp in enumerate(jacps):
            jacp_view = jacp
            mj_jac{cap_type}(self._model.ptr, self.ptr, &jacp_view[0], NULL, i)
        return jacps

    @property
    def {obj_type}_jacr(self):
        jacrs = np.zeros((self._model.n{obj_type}, 3 * self._model.nv))
        cdef double [:] jacr_view
        for i, jacr in enumerate(jacrs):
            jacr_view = jacr
            mj_jac{cap_type}(self._model.ptr, self.ptr, NULL, &jacr_view[0], i)
        return jacrs

    @property
    def {obj_type}_xvelp(self):
        jacp = self.{obj_type}_jacp.reshape((self._model.n{obj_type}, 3, self._model.nv))
        xvelp = np.dot(jacp, self.qvel)
        return xvelp

    @property
    def {obj_type}_xvelr(self):
        jacr = self.{obj_type}_jacr.reshape((self._model.n{obj_type}, 3, self._model.nv))
        xvelr = np.dot(jacr, self.qvel)
        return xvelr\n'''.format(obj_type=obj_type, cap_type=cap_type)
    return code


def _set_body_identifiers(short_name, addr_name, long_name, obj_name):
    return ("        self.{long_name}_names, self._{long_name}_name2id, self._{long_name}_id2name = "
            "self._extract_mj_names(p, p.name_{addr_name}adr, p.n{short_name}, mjtObj.mjOBJ_{obj_name})\n"
            ).format(short_name=short_name,
                     long_name=long_name,
                     obj_name=obj_name,
                     addr_name=addr_name)


def _add_getters(obj_type):
    return '''
    def {obj_type}_id2name(self, id):
        if id not in self._{obj_type}_id2name:
            raise ValueError("No {obj_type} with id %d exists." % id)
        return self._{obj_type}_id2name[id]

    def {obj_type}_name2id(self, name):
        if name not in self._{obj_type}_name2id:
            raise ValueError("No \\"{obj_type}\\" with name %s exists. Available \\"{obj_type}\\" names = %s." % (name, self.{obj_type}_names))
        return self._{obj_type}_name2id[name]
'''.format(obj_type=obj_type)


def get_const_from_define(full_src_lines):
    define_code = []
    seen = set()

    for line in full_src_lines:
        define = "#define"
        if line.find(define) > -1:
            line = line[len(define):].strip()
            last_len = 100000

            while last_len != len(line):
                last_len = len(line)
                line = line.replace("  ", " ")
                line = line.replace("\t", " ")

            comment = ""

            if line.find("//") > -1:
                line, comment = line.split("//")

            line, comment = line.strip(), comment.strip()

            if line.find(" ") > -1:
                var, val = line.split(" ")
                try:
                    # In C/C++ numbers can have an 'f' suffix, specifying a single-precision number.
                    # That is not supported by the Python floating point parser, therefore we need to strip that bit.
                    if val[-1] == 'f':
                        val = val[:-1]

                    val = float(val)
                    varname = var[2:]
                    if varname in seen:
                        print("Already seen {name}, skipping".format(name=varname))
                        continue
                    seen.add(varname)

                    new_line = varname + " = " + str(val)
                    new_line += " " * (35 - len(new_line))
                    new_line += " # " + comment
                    define_code.append(new_line)
                except Exception:
                    traceback.print_exc()
                    print("Couldn't parse line: %s" % line)

    return define_code


def get_funcs(fname):
    src = subprocess.check_output([c_compiler, '-E', '-P', fname]).decode()
    src = src[src.find("int mj_activat"):]
    l = -1

    while l != len(src):
        l = len(src)
        src = src.replace("  ", " ")
        src = src.replace("\t", " ")
        src = src.replace("\n", " ")
        src = src.replace("const ", "")
        src = src.replace(", ", ",")
        src = src.strip()

    funcs = src.split(";")
    funcs = [f.strip() for f in funcs if len(f) > 0]
    ret = ""
    count = 0

    for f in funcs:
        ret_name = f.split(" ")[0]
        func_name = f.split(" ")[1].split("(")[0]

        args = f.split("(")[1][:-1]
        skip = False

        py_args_string = []
        c_args_string = []

        if args != "void":
            args = args.split(",")

            for arg in args:
                arg = arg.strip()
                data_type = " ".join(arg.split(" ")[:-1])
                var_name = arg.split(" ")[-1]

                if var_name.find("[") > -1:
                    #arr_size = var_name[var_name.find("[") + 1:var_name.find("]")]
                    data_type = data_type + "*"
                    var_name = var_name[:var_name.find("[")]

                # Some words are keywords in Python that are not keywords in C/C++, therefore they can be used as
                # variable identifiers. We need to handle these situations.
                if var_name in ['def']:
                    var_name = '_' + var_name

                if data_type in ["char*"]:
                    py_args_string.append("str " + var_name)
                    c_args_string.append(var_name + ".encode()")
                    continue
                if data_type in ["unsigned char"]:
                    skip = True
                    break
                if data_type == "mjtNum":
                    py_args_string.append("float " + var_name)
                    c_args_string.append(var_name)
                    continue
                if data_type == "mjtNum*":
                    py_args_string.append(
                        "np.ndarray[np.float64_t, mode=\"c\", ndim=1] " + var_name)
                    c_args_string.append("&%s[0]" % var_name)
                    continue
                if data_type == "mjtByte":
                    py_args_string.append("int " + var_name)
                    c_args_string.append(var_name)
                    continue
                if data_type == "mjtByte*":
                    py_args_string.append(
                        "np.ndarray[np.uint8_t, mode=\"c\", ndim=1] " + var_name)
                    c_args_string.append("&%s[0]" % var_name)
                    continue
                if data_type[:2] == "mj" and data_type[-1] == "*":
                    py_args_string.append(
                        "PyMj" + data_type[2:-1] + " " + var_name)
                    c_args_string.append(var_name + ".ptr")
                    continue
                if data_type[:2] == 'mj' and '*' not in data_type:
                    py_args_string.append(
                        "PyMj" + data_type[2:] + " " + var_name)
                    c_args_string.append(var_name + ".ptr[0]")  # dereference
                    continue
                if data_type in "int":
                    py_args_string.append("int " + var_name)
                    c_args_string.append(var_name)
                    continue
                if data_type in "int*":
                    py_args_string.append("uintptr_t " + var_name)
                    c_args_string.append("<int*>" + var_name)
                    continue

                # XXX
                skip = True

        if not skip and ((ret_name in ["int", "mjtNum", "void"]) or
                         (ret_name[:2] == "mj" and ret_name[-1] == "*") and ret_name != "mjtNum*" and ret_name != "mjData*"):

            code = "def _%s(%s):\n" % (func_name, ", ".join(py_args_string))
            ret_val = "%s(%s)" % (func_name, ", ".join(c_args_string))
            code += "    "
            if ret_name in ["int", "mjtNum"]:
                code += "return " + ret_val
            elif ret_name == "void":
                code += ret_val
            elif ret_name[:2] == "mj":
                code += "return WrapMj" + ret_name[2:-1] + "(" + ret_val + ")"
            else:
                import ipdb
                ipdb.set_trace()
            ret += code + "\n\n"
            count += 1

    print(ret)
    print("Generated %d out of %d" % (count, len(funcs)))
    return ret


def main():
    HEADER_DIR = os.path.expanduser(os.path.join('~', '.mujoco', 'mujoco210', 'include'))
    HEADER_FILES = [
        'mjmodel.h',
        'mjdata.h',
        'mjvisualize.h',
        'mjrender.h',
        'mjui.h'
    ]
    if len(sys.argv) > 1:
        OUTPUT = sys.argv[1]
    else:
        OUTPUT = os.path.join('mujoco_py', 'generated', 'wrappers.pxi')
    OUTPUT_CONST = os.path.join('mujoco_py', 'generated', 'const.py')

    funcs = get_funcs(os.path.join(HEADER_DIR, "mujoco.h"))
    full_src_lines = get_full_scr_lines(HEADER_DIR, HEADER_FILES)
    array_shapes = get_array_shapes(full_src_lines)
    processed_src = get_processed_src(HEADER_DIR, full_src_lines)
    struct_dict = get_full_struct_dict(processed_src, array_shapes)

    structname2wrappername, structname2wrapfuncname = get_struct_wrapper(struct_dict)

    define_const = get_const_from_define(full_src_lines)
    enum_const = get_const_from_enum(processed_src)
    const_code = "# Automatically generated. Do not modify!\n\n###### const from defines ######\n"
    const_code += "\n".join(define_const)
    const_code += "\n\n###### const from enums ######\n\n"
    const_code += "\n".join(enum_const)
    with open(OUTPUT_CONST, 'w') as f:
        f.write(const_code)

    code = []
    needed_1d_wrappers = set()
    needed_2d_wrappers = set()
    # ===== Generate wrapper extension classes =====
    for name, fields in struct_dict.items():
        member_decls, member_initializers, member_getters = [], [], []

        model_var_name = 'p' if name == 'mjModel' else 'model'

        # Disabling a few accessors that are unsafe due to ambiguous meaning.
        REPLACEMENT_BY_ORIGINAL = OrderedDict([
            ('xpos', 'body_xpos'),
            ('xmat', 'body_xmat'),
            ('xquat', 'body_xquat'),
            ('efc_pos', 'active_contacts_efc_pos'),
        ])

        for scalar_name, scalar_type in fields['scalars']:
            if scalar_type in ['float', 'int', 'mjtNum', 'mjtByte', 'unsigned int']:
                member_getters.append(
                    '    @property\n    def {name}(self): return self.ptr.{name}'.format(name=scalar_name))
                member_getters.append('    @{name}.setter\n    def {name}(self, {type} x): self.ptr.{name} = x'.format(
                    name=scalar_name, type=scalar_type))
            elif scalar_type in struct_dict:
                # This is a struct member
                member_decls.append('    cdef {} _{}'.format(
                    structname2wrappername[scalar_type], scalar_name))
                member_initializers.append('        self._{scalar_name} = {wrap_func_name}(&p.{scalar_name}{model_arg})'.format(
                    scalar_name=scalar_name,
                    wrap_func_name=structname2wrapfuncname[scalar_type],
                    model_arg=(
                        ', ' + model_var_name) if struct_dict[scalar_type]['depends_on_model'] else ''
                ))
                member_getters.append(
                    '    @property\n    def {name}(self): return self._{name}'.format(name=scalar_name))
            else:
                print('Warning: skipping {} {}.{}'.format(
                    scalar_type, name, scalar_name))

        # Pointer types
        for ptr_name, ptr_type, (shape0, shape1) in fields['ptrs']:
            if ptr_type in struct_dict:
                assert shape0.startswith('n') and shape1 == 1
                member_decls.append('    cdef tuple _{}'.format(ptr_name))
                member_initializers.append(
                    '        self._{ptr_name} = tuple([{wrap_func_name}(&p.{ptr_name}[i]{model_arg}) for i in range({size0})])'.format(
                        ptr_name=ptr_name,
                        wrap_func_name=structname2wrapfuncname[ptr_type],
                        size0='{}.{}'.format(model_var_name, shape0),
                        model_arg=(
                            ', ' + model_var_name) if struct_dict[ptr_type]['depends_on_model'] else ''
                    ))
            else:
                assert name == 'mjModel' or fields['depends_on_model']
                member_decls.append('    cdef np.ndarray _{}'.format(ptr_name))
                if shape0 == 1 or shape1 == 1:
                    # Collapse to 1d for the user's convenience
                    size0 = shape1 if shape0 == 1 else shape0
                    member_initializers.append(
                        '        self._{ptr_name} = _wrap_{ptr_type}_1d(p.{ptr_name}, {size0})'.format(
                            ptr_name=ptr_name,
                            ptr_type=ptr_type.replace(' ', '_'),
                            size0=format_size_argument(model_var_name, size0),
                        ))
                else:
                    member_initializers.append(
                        '        self._{ptr_name} = _wrap_{ptr_type}_2d(p.{ptr_name}, {size0}, {size1})'.format(
                            ptr_name=ptr_name,
                            ptr_type=ptr_type.replace(' ', '_'),
                            size0=format_size_argument(model_var_name, shape0),
                            size1=format_size_argument(model_var_name, shape1),
                        ))
                needed_2d_wrappers.add(ptr_type)

            if ptr_name in REPLACEMENT_BY_ORIGINAL:
                member_getters.append("""
    @property
    def {name}(self):
        raise RuntimeError("{replacement} should be used instead of {name}")\n""".format(
                    name=ptr_name, replacement=REPLACEMENT_BY_ORIGINAL[ptr_name]))
            else:
                member_getters.append(
                    '    @property\n    def {name}(self): return self._{name}'.format(name=ptr_name))

        # Array types: handle the same way as pointers
        for array_name, array_type, array_size in fields['arrays']:
            if array_type in struct_dict:
                # This is a struct member
                member_decls.append('    cdef list _{}'.format(array_name))
                member_initializers.append('        self._{array_name} = [{wrap_func_name}(&p.{array_name}{model_arg}[i]) for i in range({array_size})]'.format(
                    array_name=array_name,
                    array_size=array_size,
                    wrap_func_name=structname2wrapfuncname[array_type],
                    model_arg=(
                        ', ' + model_var_name) if struct_dict[array_type]['depends_on_model'] else ''
                ))
                member_getters.append(
                    '    @property\n    def {name}(self): return self._{name}'.format(name=array_name))
            else:
                member_decls.append(
                    '    cdef np.ndarray _{}'.format(array_name))
                member_initializers.append(
                    '        self._{array_name} = _wrap_{array_type}_1d(&p.{array_name}[0], {size})'.format(
                        array_name=array_name,
                        array_type=array_type.replace(' ', '_'),
                        size=array_size,
                    ))
                member_getters.append(
                    '    @property\n    def {name}(self): return self._{name}'.format(name=array_name))
                needed_1d_wrappers.add(array_type)

        # 2D-Array types: handle the same way as pointers
        for array_name, array_type, array_size in fields['arrays2d']:
            if array_type in struct_dict:
                print("Skipping 2d array of structs {name}.{arr_name}: <{arr_type}[:{arr_size0},:{arr_size1}]>".format(
                    name=name,
                    arr_name=array_name,
                    arr_type=array_type,
                    arr_size0=array_size[0],
                    arr_size1=array_size[1])
                )
                continue
            else:
                member_decls.append(
                    '    cdef np.ndarray _{}'.format(array_name))
                member_initializers.append(
                    '        self._{array_name} = _wrap_{array_type}_2d(&p.{array_name}[0][0], {size0}, {size1})'.format(
                        array_name=array_name,
                        array_type=array_type.replace(' ', '_'),
                        size0=array_size[0],
                        size1=array_size[1],
                    ))
                member_getters.append(
                    '    @property\n    def {name}(self): return self._{name}'.format(name=array_name))
                needed_2d_wrappers.add(array_type)

        member_getters = '\n'.join(member_getters)
        member_decls = '\n' + '\n'.join(member_decls) if member_decls else ''
        member_initializers = '\n' + \
            '\n'.join(member_initializers) if member_initializers else ''
        model_decl = '\n    cdef PyMjModel _model' if fields[
            'depends_on_model'] else ''
        model_param = ', PyMjModel model' if fields['depends_on_model'] else ''
        model_setter = 'self._model = model' if fields[
            'depends_on_model'] else ''
        model_arg = ', model' if fields['depends_on_model'] else ''

        if name == "mjModel":
            extra = '\n'
            obj_types = ['body',
                         'joint',
                         'geom',
                         'site',
                         'light',
                         'camera',
                         'actuator',
                         'sensor',
                         'tendon',
                         'mesh']
            obj_types_names = [o + '_names' for o in obj_types]
            extra += '    cdef readonly tuple ' + ', '.join(obj_types_names) + '\n'
            obj_types_id2names = ['_' + o + '_id2name' for o in obj_types]
            extra += '    cdef readonly dict ' + ', '.join(obj_types_id2names) + '\n'
            obj_types_name2ids = ['_' + o + '_name2id' for o in obj_types]
            extra += '    cdef readonly dict ' + ', '.join(obj_types_name2ids) + '\n'
            for obj_type in obj_types:
                extra += _add_getters(obj_type)
            # Note: named userdata fields are not present in MuJoCo,
            # they're special accessors we add in mujoco-py.
            # So these fields need to be python accessible instead of readonly.
            extra += '    cdef public tuple userdata_names\n'
            extra += '    cdef public dict _userdata_id2name\n'
            extra += '    cdef public dict _userdata_name2id\n'
            extra += _add_getters('userdata')
            extra += '''
    cdef inline tuple _extract_mj_names(self, mjModel* p, int*name_adr, int n, mjtObj obj_type):
        cdef char *name
        cdef int obj_id

        # objects don't need to be named in the XML, so name might be None
        id2name = {i: None for i in range(n)}
        name2id = {}
        for i in range(n):
            name = p.names + name_adr[i]
            decoded_name = name.decode()
            if decoded_name:
                obj_id = mj_name2id(p, obj_type, name)
                assert 0 <= obj_id < n and id2name[obj_id] is None
                name2id[decoded_name] = obj_id
                id2name[obj_id] = decoded_name

        # sort names by increasing id to keep order deterministic
        return tuple(id2name[id] for id in sorted(name2id.values())), name2id, id2name

    def get_xml(self):
        cdef char errstr[300]
        cdef int ret
        with TemporaryDirectory() as td:
            filename = os.path.join(td, 'model.xml')
            with wrap_mujoco_warning():
                ret = mj_saveLastXML(filename.encode(), self.ptr, errstr, 300)
            if ret == 0:
                raise Exception('Failed to save XML: {}'.format(errstr))
            return open(filename).read()

    def get_mjb(self):
        with TemporaryDirectory() as td:
            filename = os.path.join(td, 'model.mjb')
            with wrap_mujoco_warning():
                mj_saveModel(self.ptr, filename.encode(), NULL, 0)
            return open(filename, 'rb').read()

    def set_userdata_names(self, userdata_names):
        assert isinstance(userdata_names, (list, tuple)), 'bad userdata names'
        assert len(userdata_names) <= self.nuserdata, 'insufficient userdata'
        self.userdata_names = tuple(userdata_names)
        self._userdata_id2name = dict()
        self._userdata_name2id = dict()
        for i, name in enumerate(userdata_names):
            assert isinstance(name, str), 'names must all be strings'
            self._userdata_id2name[i] = name
            self._userdata_name2id[name] = i

    def __dealloc__(self):
        mj_deleteModel(self.ptr)
'''
            extra_set = '\n'
            # MuJoCo isn't very consistent in how it uses long and
            # abbreviated names :(
            extra_set += _set_body_identifiers('body', 'body', 'body', 'BODY')
            extra_set += _set_body_identifiers('jnt', 'jnt', 'joint', 'JOINT')
            extra_set += _set_body_identifiers('geom', 'geom', 'geom', 'GEOM')
            extra_set += _set_body_identifiers('site', 'site', 'site', 'SITE')
            extra_set += _set_body_identifiers('light',
                                               'light', 'light', 'LIGHT')
            extra_set += _set_body_identifiers('cam',
                                               'cam', 'camera', 'CAMERA')
            extra_set += _set_body_identifiers('u',
                                               'actuator', 'actuator', 'ACTUATOR')
            extra_set += _set_body_identifiers('sensor',
                                               'sensor', 'sensor', 'SENSOR')
            extra_set += _set_body_identifiers('tendon',
                                               'tendon', 'tendon', 'TENDON')
            extra_set += _set_body_identifiers('mesh',
                                               'mesh', 'mesh', 'MESH')
            # userdata_names is empty at construction time
            extra_set += '        self.userdata_names = tuple()\n'
            extra_set += '        self._userdata_name2id = dict()\n'
            extra_set += '        self._userdata_id2name = dict()\n'

            for q_type in ('pos', 'vel'):
                # Position dimensionality and degrees of freedom are different
                # for free and ball joints.
                if q_type == 'pos':
                    adr_name, free_ndim, ball_ndim = 'qpos', 7, 4
                else:
                    adr_name, free_ndim, ball_ndim = 'dof', 6, 3

                extra += """
    def get_joint_q{q_type}_addr(self, name):
        '''
        Returns the q{q_type} address for given joint.

        Returns:
        - address (int, tuple): returns int address if 1-dim joint, otherwise
            returns the a (start, end) tuple for {q_type}[start:end] access.
        '''
        joint_id = self.joint_name2id(name)
        joint_type = self.jnt_type[joint_id]
        joint_addr = self.jnt_{adr_name}adr[joint_id]
        if joint_type == mjtJoint.mjJNT_FREE:
            ndim = {free_ndim}
        elif joint_type == mjtJoint.mjJNT_BALL:
            ndim = {ball_ndim}
        else:
            assert joint_type in (mjtJoint.mjJNT_HINGE, mjtJoint.mjJNT_SLIDE)
            ndim = 1

        if ndim == 1:
            return joint_addr
        else:
            return (joint_addr, joint_addr + ndim)\n""".format(
                    q_type=q_type, adr_name=adr_name,
                    free_ndim=free_ndim, ball_ndim=ball_ndim)

        elif name == "mjData":
            extra = '''
    @property
    def body_xpos(self):
        return self._xpos

    @property
    def body_xquat(self):
        return self._xquat

    @property
    def body_xmat(self):
        return self._xmat

    @property
    def active_contacts_efc_pos(self):
        return self._efc_pos[self.ne:self.nefc]

    def __dealloc__(self):
        mj_deleteData(self.ptr)

'''
            extra += _add_named_access_methods('body', 'xpos', 'xpos')
            extra += _add_named_access_methods('body', 'xquat', 'xquat')
            extra += _add_named_access_methods('body', 'xmat', 'xmat')
            extra += _add_named_access_methods('body', 'xipos', 'xipos')
            extra += _add_named_access_methods('body', 'ximat', 'ximat')
            extra += _add_named_jacobian_methods('body')
            member_getters += _add_jacobian_getters('body')
            extra += _add_named_access_methods('joint', 'xanchor', 'xanchor')
            extra += _add_named_access_methods('joint', 'xaxis', 'xaxis')
            extra += _add_named_access_methods('geom', 'geom_xpos', 'xpos')
            extra += _add_named_access_methods('geom', 'geom_xmat', 'xmat')
            extra += _add_named_jacobian_methods('geom')
            member_getters += _add_jacobian_getters('geom')
            extra += _add_named_access_methods('site', 'site_xpos', 'xpos')
            extra += _add_named_access_methods('site', 'site_xmat', 'xmat')
            extra += _add_named_jacobian_methods('site')
            member_getters += _add_jacobian_getters('site')
            extra += _add_named_access_methods('camera', 'cam_xpos', 'xpos')
            extra += _add_named_access_methods('camera', 'cam_xmat', 'xmat')
            extra += _add_named_access_methods('light', 'light_xpos', 'xpos')
            extra += _add_named_access_methods('light', 'light_xdir', 'xdir')
            extra += _add_named_access_methods('sensor', 'sensordata', None)
            extra += _add_named_access_methods('userdata', 'userdata', None)

            for pose_type in ('pos', 'quat'):
                extra += """
    def get_mocap_{pose_type}(self, name):
        body_id = self._model.body_name2id(name)
        mocap_id = self._model.body_mocapid[body_id]
        return self.mocap_{pose_type}[mocap_id]

    def set_mocap_{pose_type}(self, name, value):
        body_id = self._model.body_name2id(name)
        mocap_id = self._model.body_mocapid[body_id]
        self.mocap_{pose_type}[mocap_id] = value\n""".format(
                    pose_type=pose_type)

            for q_type in ('pos', 'vel'):
                extra += """
    def get_joint_q{q_type}(self, name):
        addr = self._model.get_joint_q{q_type}_addr(name)
        if isinstance(addr, (int, np.int32, np.int64)):
            return self.q{q_type}[addr]
        else:
            start_i, end_i = addr
            return self.q{q_type}[start_i:end_i]

    def set_joint_q{q_type}(self, name, value):
        addr = self._model.get_joint_q{q_type}_addr(name)
        if isinstance(addr, (int, np.int32, np.int64)):
            self.q{q_type}[addr] = value
        else:
            start_i, end_i = addr
            value = np.array(value)
            assert value.shape == (end_i - start_i,), (
                "Value has incorrect shape %s: %s" % (name, value))
            self.q{q_type}[start_i:end_i] = value\n""".format(
                    q_type=q_type)

            extra_set = ""
        elif name in ["mjVFS", "mjrRect"]:
            extra = '''
    def __cinit__(self):
        self.ptr = <{name}*> PyMem_Malloc(sizeof({name}))
        if not self.ptr:
            raise MemoryError()

    def __dealloc__(self):
        PyMem_Free(self.ptr)
'''.format(name=name)
            extra_set = ''
        elif name in [
            'mjuiItemSingle', 'mjuiItemMulti', 'mjuiItemSlider', 'mjuiItemEdit'
        ]:
          # these structs don't have a corresponding typedef.
          continue
        elif name[:2] == 'mj':
            extra = '''
    def __cinit__(self):
        self.ptr = NULL
'''
            extra_set = ''
        else:
            extra = ""
            extra_set = ""

        code.append('''
cdef class {wrapper_name}(object):
    cdef {struct_name}* ptr
    {model_decl}
    {member_decls}
    {extra}

    @property
    def uintptr(self): return <uintptr_t>self.ptr

    cdef void _set(self, {struct_name}* p{model_param}):
        {extra_set}
        self.ptr = p
        {model_setter}
        {member_initializers}
        \n{member_getters}

cdef {wrapper_name} {wrap_func_name}({struct_name}* p{model_param}):
    cdef {wrapper_name} o = {wrapper_name}()
    o._set(p{model_arg})
    return o

    '''.format(
            wrapper_name=structname2wrappername[name],
            extra=extra,
            extra_set=extra_set,
            struct_name=name,
            wrap_func_name=structname2wrapfuncname[name],
            model_decl=model_decl,
            model_param=model_param,
            model_setter=model_setter,
            model_arg=model_arg,
            member_decls=member_decls,
            member_initializers=member_initializers,
            member_getters=member_getters,
        ).strip())

    # ===== Generate array-to-NumPy wrappers =====
    # TODO: instead of returning None for empty arrays, instead return NumPy arrays with the appropriate shape and type
    # The only reason we're not doing this already is that cython's views don't work with 0-length axes,
    # even though NumPy does.
    # TODO: set NumPy array type explicitly (e.g. char will be viewed
    # incorrectly as np.int64)
    for type_name in sorted(needed_1d_wrappers):
        code.append('''
cdef inline np.ndarray _wrap_{type_name_nospaces}_1d({type_name}* a, int shape0):
    if shape0 == 0: return None
    cdef {type_name}[:] b = <{type_name}[:shape0]> a
    return np.asarray(b)
'''.format(type_name_nospaces=type_name.replace(' ', '_'), type_name=type_name).strip())

    for type_name in sorted(needed_2d_wrappers):
        code.append('''
cdef inline np.ndarray _wrap_{type_name_nospaces}_2d({type_name}* a, int shape0, int shape1):
    if shape0 * shape1 == 0: return None
    cdef {type_name}[:,:] b = <{type_name}[:shape0,:shape1]> a
    return np.asarray(b)
'''.format(type_name_nospaces=type_name.replace(' ', '_'), type_name=type_name).strip())

    header = '''# cython: language_level=3
# Automatically generated. Do not modify!

include "../pxd/mujoco.pxd"
from libc.stdint cimport uintptr_t
from cpython.mem cimport PyMem_Malloc, PyMem_Free
cimport numpy as np
import numpy as np
from tempfile import TemporaryDirectory

'''
    code.append(funcs)

    code = header + '\n\n'.join(code) + '\n'
    print(len(code.splitlines()))
    with open(OUTPUT, 'w') as f:
        f.write(code)


if __name__ == "__main__":
    main()
