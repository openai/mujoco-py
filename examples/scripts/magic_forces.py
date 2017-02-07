import sys
from os.path import dirname, abspath

sys.path.append(dirname(dirname(abspath(__file__))))

from mujoco_model import MujocoEnv
import random
import numpy as np

class applyFTExample():   
    def __init__(self):
        self.setup = MujocoEnv('table_setup.xml')
        self.model = self.setup.model

    def resetBox(self):
        self.model.setPos(self.model, np.array([0.25,0.,0.15,1.,0.,0.,0.]))
        self.model.setVel(self.model, np.zeros(6))
        self.model.data.qfrc_applied = np.zeros((6,1))
        self.model.forward()

    def applyFTOnObj(self):

        site_names = self.model.site_names

        if not site_names:

            print "No sites found to apply inputs"
            raise ValueError


        point1_index = random.randint(0, len(site_names)-1)
        point2_index = random.randint(0, len(site_names)-1)

        point1 = self.model.site_pose(site_names[point1_index])[0]
        point2 = self.model.site_pose(site_names[point2_index])[0]
        com    = self.model.data.xipos[1]

        f_direction1 = (com-point1)/np.linalg.norm(com-point1)
        f_direction2 = (com-point2)/np.linalg.norm(com-point2)

        force1 = 500.*f_direction1# magnitude times direction
        force2 = 500.*f_direction2#

        torque = np.random.randn(3)

        self.setup.applyFTOnObj(body_name='Box', point=com, force=force1+force2, torque=torque)

if __name__ == "__main__":
    myBox = applyFTExample()
    myBox.setup.viewerSetup()
    myBox.viewer = myBox.setup.viewerStart()

    while True:
        myBox.setup.viewerRender()
        myBox.applyFTOnObj()
        myBox.setup.step()

    myBox.setup.viewerEnd()
