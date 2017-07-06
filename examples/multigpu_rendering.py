"""
This is an example for rendering on multiple GPUs in parallel,
using the multiprocessing module.
"""
import ctypes
from multiprocessing import (
    Process, set_start_method, freeze_support, Condition,
    Value, Array)
from time import perf_counter
import numpy as np
from mujoco_py import load_model_from_path, MjSim


#
# Experiment parameters
#

# Image size for rendering
IMAGE_WIDTH = 255
IMAGE_HEIGHT = 255
# Number of frames to render per sim
N_FRAMES = 2000
# Number of sims to run in parallel (assumes one per GPU),
# so N_SIMS=2 assumes there are 2 GPUs available.
N_SIMS = 2


class RenderProcess:
    """
    Wraps a multiprocessing.Process for rendering. Assumes there
    is one MjSim per process.
    """

    def __init__(self, device_id, setup_sim, update_sim, output_var_shape):
        """
        Args:
        - device_id (int): GPU device to use for rendering (0-indexed)
        - setup_sim (callback): callback that is given a device_id and
            returns a MjSim. It is responsible for making MjSim render
            to given device.
        - update_sim (callback): callback given a sim and device_id, and
            should return a numpy array of shape `output_var_shape`.
        - output_var_shape (tuple): shape of the synchronized output
            array from `update_sim`.
        """
        self.device_id = device_id
        self.setup_sim = setup_sim
        self.update_sim = update_sim

        # Create a synchronized output variable (numpy array)
        self._shared_output_var = Array(
            ctypes.c_double, int(np.prod(output_var_shape)))
        self._output_var = np.frombuffer(
            self._shared_output_var.get_obj())

        # Number of variables used to communicate with process
        self._cv = Condition()
        self._ready = Value('b', 0)
        self._start = Value('b', 0)
        self._terminate = Value('b', 0)

        # Start the actual process
        self._process = Process(target=self._run)
        self._process.start()

    def wait(self):
        """ Wait for process to be ready for another update call. """
        with self._cv:
            if self._start.value:
                self._cv.wait()
            if self._ready.value:
                return
            self._cv.wait()

    def read(self, copy=False):
        """ Reads the output variable. Returns a copy if copy=True. """
        if copy:
            with self._shared_output_var.get_lock():
                return np.copy(self._output_var)
        else:
            return self._output_var

    def update(self):
        """ Calls update_sim asynchronously. """
        with self._cv:
            self._start.value = 1
            self._cv.notify()

    def stop(self):
        """ Tells process to stop and waits for it to terminate. """
        with self._cv:
            self._terminate.value = 1
            self._cv.notify()
        self._process.join()

    def _run(self):
        sim = self.setup_sim(self.device_id)

        while True:
            with self._cv:
                self._ready.value = 1
                self._cv.notify_all()

            with self._cv:
                if not self._start.value and not self._terminate.value:
                    self._cv.wait()
                if self._terminate.value:
                    break
                assert self._start.value
                self._start.value = 0

            # Run the update and assign output variable
            with self._shared_output_var.get_lock():
                self._output_var[:] = self.update_sim(
                    sim, self.device_id).ravel()


def setup_sim(device_id):
    model = load_model_from_path("xmls/fetch/main.xml")
    sim = MjSim(model)

    image = sim.render(
        IMAGE_WIDTH, IMAGE_HEIGHT, device_id=device_id)
    assert image.shape == (IMAGE_HEIGHT, IMAGE_WIDTH, 3)

    return sim


def update_sim(sim, device_id):
    sim.step()
    return sim.render(IMAGE_WIDTH, IMAGE_HEIGHT, device_id=device_id)


def main():
    print("main(): create processes", flush=True)
    processes = []
    for device_id in range(N_SIMS):
        p = RenderProcess(
            device_id, setup_sim, update_sim,
            (IMAGE_HEIGHT, IMAGE_WIDTH, 3))
        processes.append(p)

    for p in processes:
        p.wait()

    print("main(): start benchmarking", flush=True)
    start_t = perf_counter()

    for _ in range(N_FRAMES):
        for p in processes:
            p.update()

        for p in processes:
            p.wait()

        for p in processes:
            p.read()

    t = perf_counter() - start_t
    print("Completed in %.1fs: %.3fms, %.1f FPS" % (
            t, t / (N_FRAMES * N_SIMS) * 1000, (N_FRAMES * N_SIMS) / t),
          flush=True)

    print("main(): stopping processes", flush=True)
    for p in processes:
        p.stop()

    print("main(): finished", flush=True)


if __name__ == "__main__":
    set_start_method('spawn')
    main()
