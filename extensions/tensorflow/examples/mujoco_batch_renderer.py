"""
Example for how to use the MjBatchRender with the cuda_buffer_ops.

    python mujoco_batch_renderer.py

To save images run the following (warning, a lot of images will be
written by default into the current directory.

    python mujoco_batch_renderer.py 1
"""
import sys
from time import perf_counter

import numpy as np
import tensorflow as tf
from mujoco_py import load_model_from_xml, MjSim, MjBatchRenderer
from PIL import Image
from pycuda.driver import Context

from cuda_buffer_ops import read_cuda_buffer_uint8

BASIC_MODEL_XML = """
<mujoco>
    <worldbody>
        <light name="light1" diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <camera name="camera1" pos="3 0 0" zaxis="1 0 0" />
        <geom name="g1" pos="0.5 0.4 0.3" type="plane" size="1 1 0.1" rgba="1 0 0 1" />
        <body pos="0 0 1" name="body1">
            <joint name="joint1" type="free"/>
            <geom name="g2" pos="0 1 0" type="box" size=".1 .2 .3" rgba="0 1 0 1" />
        </body>
    </worldbody>
    <visual>
        <quality offsamples="0"/>
    </visual>
</mujoco>
"""


def main(save_images):
    batch_size = 100
    n_batches = 10

    image_width = 225
    image_height = 255

    print("Setting up MuJoCo model")
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)

    print("Creating MjBatchRenderer")
    renderer = MjBatchRenderer(
        sim, image_width, image_height, batch_size=batch_size, use_cuda=True)

    print("Collecting states to set later")
    states = []
    for i in range(batch_size * n_batches):
        sim.data.qpos[:3] = 0.1 / (batch_size * n_batches) * i
        sim.forward()
        states.append(sim.get_state())

    conf = tf.ConfigProto(
        intra_op_parallelism_threads=1,
        inter_op_parallelism_threads=1)
    with tf.Session(config=conf) as sess:
        images_tensor = read_cuda_buffer_uint8(
            renderer._cuda_rgb_ptr, image_width, image_height,
            num_images=batch_size)

        print("Running benchmark")
        t_set_state, t_forward, t_render, t_copy, t_run_session = [], [], [], [], []
        images = []
        for batch_i in range(n_batches):
            for i in range(batch_size):
                idx = batch_i * batch_size + i

                _t = perf_counter()
                sim.set_state(states[idx])
                t_set_state.append(perf_counter() - _t)

                _t = perf_counter()
                sim.forward()
                t_forward.append(perf_counter() - _t)

                _t = perf_counter()
                renderer.render()
                t_render.append(perf_counter() - _t)

            _t = perf_counter()
            renderer.copy_gpu_buffers()
            t_copy.append(perf_counter() - _t)

            _t = perf_counter()
            (tf_images,) = sess.run([images_tensor])
            t_run_session.append(perf_counter() - _t)
            assert tf_images.shape == (batch_size, image_height, image_width, 3)

            for img in tf_images:
                images.append(img)

    def print_timing(var, var_name):
        print("- %s: mean=%.0f p50=%.0f p99=%.0f (µs)" % (
            var_name,
            np.mean(var) * 1e6,
            np.percentile(var, 50) * 1e6,
            np.percentile(var, 99) * 1e6,
        ))

    print("Benchmark completed:")
    print_timing(t_set_state, 'set_state')
    print_timing(t_forward, 'forward')
    print_timing(t_render, 'render')
    print_timing(t_copy, 'copy')
    print_timing(t_run_session, 'run_session')

    if save_images:
        print("Writing images...")
        for j, image in enumerate(images):
            Image.fromarray(image).save('cuda_image_%04d.png' % j)

    Context.pop()
    print("Done")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        save_images = bool(sys.argv[1])
    else:
        save_images = False

    main(save_images)
