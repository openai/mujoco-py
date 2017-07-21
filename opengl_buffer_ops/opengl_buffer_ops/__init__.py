from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os.path
import tensorflow as tf

data_files_path = tf.resource_loader.get_data_files_path()
_op_module = tf.load_op_library(
    os.path.join(data_files_path, 'opengl_buffer_ops.so'))

read_gl_buffer_op = _op_module.read_gl_buffer


def read_gl_buffer(handle, width, height, num_images=1, const_handle=True):
    if const_handle:
        dummy_handle = tf.constant(0, dtype=tf.int32)
        return read_gl_buffer_op(
            dummy_handle, width=width, height=height, num_images=num_images,
            const_handle=handle)
    else:
        return read_gl_buffer_op(
            handle, width=width, height=height, num_images=num_images)
