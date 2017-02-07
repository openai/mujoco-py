from os.path import dirname, abspath

model_folder_path = dirname(dirname(abspath(__file__))) + '/simulation/models/'

#barret hand model_folder_path+'/barrett/bhand.xml',

MUJOCO_ENV = {
	
	'image_width': 640,
	'image_height': 480,
	'pixels_per_meter': 20.0,
	'fps': 60,
	'box_dim':(0.15, 0.15, 0.15),
	'model_name':model_folder_path+'turning_box.xml', #model_folder_path+'table_setup.xml'
	'dt': 0.0167,
	'window_caption': 'MujocoWorld',
	'record_training_data': True,
	'training_data_file': 'data_test.pkl',
	'steps_per_frame': 5,
	'camera_pos': [0.0, -0.4, 0.65, 1., -45., 45.],
	
}