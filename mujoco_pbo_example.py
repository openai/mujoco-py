from time import perf_counter
from mujoco_py import load_model_from_xml, MjSim, MjBatchRenderer
from PIL import Image

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

def get_renderer(batch_size, image_size):
    model = load_model_from_xml(BASIC_MODEL_XML)
    sim = MjSim(model)
    renderer = MjBatchRenderer(sim, image_size, image_size, batch_size=batch_size)
    return sim, renderer


def render_batch(sim, renderer, batch_size, vel=1):
    print("rendering:")
    for pos in range(batch_size):
        sim.data.qpos[:3] = pos * 0.03 * vel
        sim.forward()

        t = perf_counter()
        renderer.render()
        print("  rendered in %.0f us" % ((perf_counter() - t) * 1e6))


def main_cpu(batch_size=10):
    image_size = 255
    sim, renderer = get_renderer(batch_size, image_size)
    render_batch(sim, renderer, batch_size)

    t = perf_counter()
    images = renderer.read()
    print("read images in %.0f ms" % ((perf_counter() - t) * 1e6))

    print("shape:", images.shape)
    print("sum:", images.ravel().sum())

    for i, image in enumerate(images):
        Image.fromarray(image).save('image_%02d.png' % i)


def main_gpu(batch_size=10):
    import numpy as np
    print("START")

    image_size = 255
    sim, renderer = get_renderer(batch_size, image_size)
    render_batch(sim, renderer, batch_size)

    import pycuda.autoinit
    import pycuda.driver as drv
    from pycuda.gl import make_context, RegisteredBuffer
    device = pycuda.autoinit.device
    buf_size = batch_size * image_size * image_size * 3
    cuda_buf = drv.mem_alloc(buf_size)


    # make_context(device)  # need to do after OpenGL context

    cuda_pbo = RegisteredBuffer(renderer.pbo)
    mapping = cuda_pbo.map()
    buf_ptr, _ = mapping.device_ptr_and_size()

    drv.memcpy_dtod(cuda_buf, buf_ptr, buf_size)

    images = drv.from_device(buf_ptr,
                             shape=(batch_size, image_size, image_size, 3),
                             dtype=np.uint8)

    for i, image in enumerate(images):
        Image.fromarray(image).save('cuda_image_%02d.png' % i)

    mapping.unmap()
    cuda_pbo.unregister()
    print("DONE!")


def main_tf(batch_size=10):
    import tensorflow as tf
    from opengl_buffer_ops import read_gl_buffer

    print("START")
    image_size = 255

    sim, renderer = get_renderer(batch_size, image_size)
    render_batch(sim, renderer, batch_size)

    conf = tf.ConfigProto(
        intra_op_parallelism_threads=1,
        inter_op_parallelism_threads=1)

    sess = tf.Session(config=conf)
    # tf_pbo_handle = tf.constant(renderer.pbo, dtype=tf.int32)
    tf_pbo_handle = renderer.pbo
    print("XXX renderer.pbo", renderer.pbo)

    context_pointer = renderer.render_context._opengl_context.get_context_pointer()
    print("Python got context pointer", context_pointer)
    display_pointer = renderer.render_context._opengl_context.get_display_pointer()
    print("Python got display pointer", display_pointer)

    images_tensor = read_gl_buffer(
        tf_pbo_handle, image_size, image_size, display_pointer, num_images=batch_size,
        const_handle=True)
    images = sess.run([images_tensor])

    for i, image in enumerate(images):
        Image.fromarray(image).save('cuda_image_%02d.png' % i)

    print("DONE!")


def main_tf_and_pycuda(batch_size=10):
    image_size = 255
    sim, renderer = get_renderer(batch_size, image_size)

    import numpy as np
    import pycuda.autoinit
    import pycuda.driver as drv
    from pycuda.gl import RegisteredBuffer
    device = pycuda.autoinit.device
    buf_size = batch_size * image_size * image_size * 3
    cuda_buf = drv.mem_alloc(buf_size)

    cuda_pbo = RegisteredBuffer(renderer.pbo)
    mapping = cuda_pbo.map()
    buf_ptr, _ = mapping.device_ptr_and_size()
    print("Buffer pointer", buf_ptr)

    import tensorflow as tf
    from opengl_buffer_ops import read_gl_buffer
    conf = tf.ConfigProto(
        intra_op_parallelism_threads=1,
        inter_op_parallelism_threads=1)
    sess = tf.Session(config=conf)

    for i in range(2):
        t = perf_counter()
        render_batch(sim, renderer, batch_size, vel=0.5 * (i + 1))
        print(">>> render_batch %.1f us" % ((perf_counter() - t) * 1e6))
        t = perf_counter()
        drv.memcpy_dtod(cuda_buf, buf_ptr, buf_size)
        print(">>> memcpy_dtod %.1f us" % ((perf_counter() - t) * 1e6))
        t = perf_counter()

        images_tensor = read_gl_buffer(
            buf_ptr, image_size, image_size, num_images=batch_size, const_handle=True)
        (tf_images,) = sess.run([images_tensor])
        print("Image sum:", tf_images.ravel().sum())

        for j, image in enumerate(tf_images):
            Image.fromarray(image).save('cuda_image_%02d_%02d.png' % (j, i))

    mapping.unmap()
    cuda_pbo.unregister()

    print("Done")


def main_egl_context(batch_size=10):
    image_size = 255
    sim, renderer = get_renderer(batch_size, image_size)
    context_pointer = renderer.render_context._opengl_context.get_context_pointer()
    print("Python got context pointer", context_pointer)
    display_pointer = renderer.render_context._opengl_context.get_display_pointer()
    print("Python got display pointer", display_pointer)


if __name__ == "__main__":
    # main_tf()
    # main_egl_context()
    main_tf_and_pycuda()
