from mujoco_py import load_model_from_path, MjSim
from mujoco_py.mjviewer import MjViewer
from mujoco_py.tests.utils import requires_rendering


@requires_rendering
def test_viewer():
    model = load_model_from_path("mujoco_py/tests/test.xml")
    sim = MjSim(model)
    viewer = MjViewer(sim)
    for _ in range(100):
        sim.step()
        viewer.render()
