import sys
from os.path import dirname, abspath

sys.path.append(dirname(dirname(abspath(__file__))))

from mujoco_model import MujocoEnv
import numpy as np

class tendonExample():   
    def __init__(self):
        self.setup = MujocoEnv('box_with_tendon.xml')
        self.model = self.setup.model

    def applyRandCtrl(self):
        self.setup.setControl(np.random.randn(self.model.nv)[:,None])

if __name__ == "__main__":
    myTend = tendonExample()
    myTend.setup.viewerSetup()
    myTend.viewer = myTend.setup.viewerStart()

    #to know contacts
    # print myTend.model.data_ptr.contents.contact.contents.dim
    # print myTend.model.data_ptr.contents.contact.contents.geom1
    # print myTend.model.data_ptr.contents.contact.contents.geom2
    

    while True:
        myTend.setup.viewerRender()
        myTend.applyRandCtrl()
        myTend.setup.step()

    myTend.setup.viewerEnd()