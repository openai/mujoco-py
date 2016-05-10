import mujoco_py
import six

def test_smoke():
    model = mujoco_py.MjModel('tests/models/ant.xml')

    # Try stepping
    model.step()

    model._compute_subtree()

    # Try getting some data out of the model
    n = model.body_names
    idx = n.index(six.b('torso'))
    com = model.data.com_subtree[idx]
