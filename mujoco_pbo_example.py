from time import perf_counter, sleep
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

model = load_model_from_xml(BASIC_MODEL_XML)
sim = MjSim(model)
image_size = 255

batch_size = 10
renderer = MjBatchRenderer(sim, image_size, image_size, batch_size=batch_size)

print("rendering:", image_size)
images = []
for pos in range(batch_size):
    sim.data.qpos[:3] = pos * 0.03
    sim.forward()

    t = perf_counter()
    renderer.render()
    print("  rendered in %.0f us" % ((perf_counter() - t) * 1e6))

t = perf_counter()
images = renderer.read()
print("read images in %.0f ms" % ((perf_counter() - t) * 1e6))

print("shape:", images.shape)
print("sum:", images.ravel().sum())

for i, image in enumerate(images):
    Image.fromarray(image).save('image_%02d.png' % i)

