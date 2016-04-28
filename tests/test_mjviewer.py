"""
Test mujoco viewer.
"""
import unittest

from mujoco_py import mjviewer, mjcore


class MjLibTest(unittest.TestCase):
    xml_path = 'tests/models/cartpole.xml'

    def setUp(self):
        self.width = 100
        self.height = 100
        self.viewer = mjviewer.MjViewer(visible=False,
                                        init_width=self.width,
                                        init_height=self.height)

    def tearDown(self):
        self.viewer.finish()
        self.viewer = None

    def test_start(self):
        self.viewer.start()
        self.assertTrue(self.viewer.running)

    def test_render(self):
        self.viewer.start()

        model = mjcore.MjModel(self.xml_path)
        self.viewer.set_model(model)

        (data, width, height) = self.viewer.get_image()

        # check image size is consistent
        # note that width and height may not equal self.width and self.height
        # e.g. on a computer with retina screen,
        # the width and height are scaled
        self.assertEqual(len(data), 3 * width * height)
        # make sure the image is not pitch black
        self.assertTrue(any(map(lambda x: x > 0, data)))
