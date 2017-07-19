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


def render_batch(sim, renderer, batch_size):
    print("rendering:")
    for pos in range(batch_size):
        sim.data.qpos[:3] = pos * 0.03
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
    import pycuda.autoinit
    from pycuda.driver import from_device
    from pycuda.gl import make_context, RegisteredBuffer

    device = pycuda.autoinit.device

    image_size = 255
    sim, renderer = get_renderer(batch_size, image_size)
    render_batch(sim, renderer, batch_size)

    make_context(device)  # need to do after OpenGL context

    cuda_pbo = RegisteredBuffer(renderer.pbo)
    mapping = cuda_pbo.map()
    buf_ptr, buf_size = mapping.device_ptr_and_size()
    assert buf_size == (batch_size * image_size * image_size * 3)
    images = from_device(buf_ptr,
                         shape=(batch_size, image_size, image_size, 3),
                         dtype=np.uint8)

    for i, image in enumerate(images):
        Image.fromarray(image).save('cuda_image_%02d.png' % i)

    mapping.unmap()
    cuda_pbo.unregister()


if __name__ == "__main__":
    main_gpu()
