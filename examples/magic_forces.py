import os
import mujoco_py
from mujoco_py import mjviewer, mjcore
from mujoco_py import mjtypes
from mujoco_py import glfw
import numpy as np
import ctypes
from os import path
from os.path import dirname, abspath
import six
from math import *
import random

model_folder_path = dirname(dirname(abspath(__file__))) + '/examples/models/'

class tableScenario():   
    def __init__(self):
        self.xml_path = model_folder_path + 'table_setup.xml' 
        if not path.exists(self.xml_path):
            raise IOError("File %s does not exist"%self.xml_path)
        self.model = mjcore.MjModel(self.xml_path)
        self.dt = self.model.opt.timestep;
        #self.action_space = spaces.Box(self.lower, self.upper)
        self.metadata = {'render.modes': ['human', 'rgb_array'],
            'video.frames_per_second' : int(np.round(1.0 / self.dt))}

    def viewerSetup(self):
        self.width = 640
        self.height = 480
        self.viewer = mjviewer.MjViewer(visible=True,
                                        init_width=self.width,
                                        init_height=self.height)
        #self.viewer.cam.trackbodyid = 0 #2
        self.viewer.cam.distance = self.model.stat.extent * 0.75
        self.viewer.cam.lookat[0] = 0 #0.8
        self.viewer.cam.lookat[1] = 0.5 #0.8
        self.viewer.cam.lookat[2] = 0.1 #0.8
        self.viewer.cam.elevation = 160
        self.viewer.cam.azimuth = 100
        #self.viewer.cam.pose = 
        self.viewer.cam.camid = -3
        self.viewer.start()
        self.viewer.set_model(self.model)
        #(data, width, height) = self.viewer.get_image()

    def viewerEnd(self):
        self.viewer.finish()
        self.viewer = None

    def viewerStart(self):
        if self.viewer is None:
            self.viewerSetup()
        return self.viewer       

    def viewerRender(self):
 
        self.viewerStart().loop_once()
                                               
    def resetModel(self):
        self.model.resetData()
        ob = self.resetModel()
        if self.viewer is not None:
            self.viewer.autoscale()
            self.viewerSetup()
        return ob
        
    def getComPos(self):
        ridx = self.model.body_names.index(six.b(body_name))
        return self.model.data.com_subtree[idx]
    
    def getComVel(self, body_name):
        idx = self.model.body_names.index(six.b(body_name))
        return self.model.body_comvels[idx]   
    
    def getXmat(self, body_name):
        idx = self.model.body_names.index(six.b(body_name))
        return self.model.data.xmat[idx].reshape((3, 3))
    
    def getStateVector(self, model):
        return np.concatenate([model.data.qpos.flat,
                               model.data.qvel.flat])
        
    def setPos(self, model, q):
        #print model.data.qpos
        model.data.qpos = q
        model._compute_subtree() 
        model.forward()
        #print model.data.qpos
        return model
    
    def setVel(self, model, dq):
        model.data.qvel = dq
        model._compute_subtree() 
        model.forward()
        return model
        
    def setControl(self, model, ctrl, nFrames=1):
        model.data.ctrl = ctrl
        for _ in range(nFrames):
            model.step()
        return model
    

    def resetBox(self):
        self.setPos(self.model, np.array([0.25,0.,0.15,1.,0.,0.,0.]))
        self.setVel(self.model, np.zeros(6))
        self.qfrc_applied = np.zeros((6,1))
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

        self.model.data.qfrc_applied = np.hstack([force1+force2, torque])

    def drawCube(self):
        pos  = [0,0,0];
        ori  = [0,0,0];
        size = [100,100,100];
        self.viewer.drawCube(pos, ori, size)

if __name__ == "__main__":
    myBox = tableScenario()
    myBox.viewerSetup()
    saveData = False
    myBox.viewer = myBox.viewerStart()

    while True:
        myBox.drawCube()
        myBox.viewerRender()
        # myBox.resetBox()
        # myBox.applyFTOnObj()
        # for j in range(100):
        #     myBox.viewerRender()
        #     myBox.model.step()
    myBox.viewerEnd()
