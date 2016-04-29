import mujoco_py

def test_smoke():
    model = mujoco_py.MjModel('tests/models/cartpole.xml')
    model.step()

    model.body_names
