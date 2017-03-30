from mujoco_py import mjviewer, mjcore
from mujoco_py import mjtypes
from mujoco_py import glfw

def test_image_size():
    width = 400
    height = 300
    viewer = mjviewer.MjViewer(visible=True, init_width=width, init_height=height)
    viewer.start()
    model = mjcore.MjModel('tests/models/cartpole.xml')
    viewer.set_model(model)
    (data, width, height) = viewer.get_image()
    assert (width==400 and height==300) or (width==800 and height==600)
    viewer.finish()
    viewer = None
