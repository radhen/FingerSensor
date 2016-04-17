import rospy
import baxter_interface
from baxter_pykdl import baxter_kinematics
import numpy as np
import time

rospy.init_node("test")
limb_name = 'left'
other_limb_name = 'right'
limb = baxter_interface.Limb(limb_name)

def jointvelocities(velocity):
	jacobianinv = baxter_kinematics(limb_name).jacobian_pseudo_inverse()
	v = np.array(jacobianinv.dot(velocity))[0]
	dict2 = {'head_nod':0.0 , 'head_pan':0.0, 'torso_t0':0.0}
	joint_vel_dict = {'{}_{}'.format(limb_name, joint_name): val for joint_name, val in zip(['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2'], v)}
	joint_vel_dict.update({'{}_{}'.format(other_limb_name, joint_name): 0.0 for joint_name, val in zip(['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2'], v)})
	joint_vel_dict.update(dict2)
	limb.set_joint_velocities(joint_vel_dict)

velocity = [0.0, 0.005, 0, 0, 0, 0];
while True:
	jointvelocities(velocity)
