import mujoco_py
from mujoco_py import mjviewer
from mujoco_py import mjcore
from mujoco_py import mjtypes
from mujoco_py import glfw
import numpy as np
from os.path import dirname, abspath
from config import MUJOCO_ENV

model_folder_path = dirname(dirname(abspath(__file__))) + '/examples/models/'

class MujocoEnv():   
    def __init__(self, model_name, config=MUJOCO_ENV):
        self.config = config
        self.xml_path = model_folder_path + model_name
        self.model = mjcore.MjModel(self.xml_path)
        self.dt = self.model.opt.timestep;

    def viewerSetup(self):

        self.viewer = mjviewer.MjViewer(visible=True,
                                        init_width=self.config['image_width'],
                                        init_height=self.config['image_height'])

        self.viewer.start()
        self.viewer.set_model(self.model)

        if 'camera_pos' in self.config:
            cam_pos = self.config['camera_pos']
            for i in range(3):
                self.viewer.cam.lookat[i] = cam_pos[i]
            self.viewer.cam.distance      = cam_pos[3]
            self.viewer.cam.elevation     = cam_pos[4]
            self.viewer.cam.azimuth       = cam_pos[5]
            self.viewer.cam.trackbodyid   = -1 

    def get_current_image(self):
        (data, width, height) = self.viewer.get_image()
        return data

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
          
    def getPos(self):
        return self.model.data.qpos.flatten()

    def getVel(self):
        return self.model.data.qvel.flatten()

    def setPos(self, pos):
        self.model.data.qpos = pos

    def setVel(self, vel):
        self.model.data.qvel = vel
        
    def setControl(self, ctrl):
        self.model.data.ctrl = ctrl

    def step(self):
        self.model.step()
        self.viewerRender()  

    def resetBox(self):
        self.setPos(np.zeros((self.box.model.nq,1)))
        self.resetVel()
        self.model.qfrc_applied = np.zeros((self.model.nv,1))

    def resetVel(self):
        self.setVel(np.zeros((self.box.model.nv,1)))
        self.model.step1()

    def applyFTOnObj(self, body_name, point, force, torque=np.zeros(3)):

        qfrc_target = self.model.applyFT(point  = point,
                                         force  = force, 
                                         torque = torque, body_name=body_name)
        self.model.data.qfrc_applied = qfrc_target
