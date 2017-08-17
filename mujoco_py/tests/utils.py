import os
from PIL import Image
from os.path import exists, join, dirname, abspath
from shutil import move
import numpy as np
from os.path import splitext
import imagehash
import pytest

if os.getenv('MUJOCO_PY_TEST_ASSET_DIR_PATH'):
    TEST_ASSET_DIR_PATH = os.getenv('MUJOCO_PY_TEST_ASSET_DIR_PATH')
else:
    TEST_ASSET_DIR_PATH = abspath(join(dirname(__file__), '..', 'test_imgs'))


def save_test_image(filename, array):
    Image.fromarray(array).save(filename)


def compare_imgs(img, truth_filename, do_assert=True):
    """
    PROTIP: run the following to re-generate the test images:

        REGENERATE_TEST_IMAGES=1 pytest mujoco_py/tests/test_modder.py

    Note: do this in Docker so that images will work for testing.
    """
    assert isinstance(truth_filename, str)
    truth_filename = join(TEST_ASSET_DIR_PATH, truth_filename)
    if os.getenv('REGENERATE_TEST_IMAGES'):
        if exists(truth_filename):
            pre_path, ext = splitext(truth_filename)
            backup_path = "%s_old%s" % (pre_path, ext)
            move(truth_filename, backup_path)
        save_test_image(truth_filename, img)
        return 0
    true_img = np.asarray(Image.open(truth_filename))
    assert img.shape == true_img.shape
    hash0 = imagehash.dhash(Image.fromarray(img))
    hash1 = imagehash.dhash(Image.fromarray(true_img))
    diff = np.sum(hash0.hash != hash1.hash)
    if diff != 0:
        # If the assert fails, the best way to investigate is to run
        # pytest for the particular test. For example,
        #
        #       pytest -k test_something_something path/to/test.py
        save_test_image("/tmp/img.png", img)
        save_test_image("/tmp/true_img.png", true_img)
        save_test_image("/tmp/diff_img.png", img - true_img)
    if do_assert:
        assert diff <= 1
    return diff
