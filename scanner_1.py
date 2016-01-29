import rospy
import baxter_interface
import numpy as np
import serial
import re

rospy.init_node('object_scanner_1')
#limb = baxter_interface.Limb('left')
leftgripper = baxter_interface.Gripper('left')
#angles = limb.joint_angles()

#Initalize robot scanning position
#angles['left_s0']=-0.1917
#angles['left_s1']=0.2316
#angles['left_e0']=-0.3788
#angles['left_e1']=0.68837
#angles['left_w0']=-0.9641
#angles['left_w1']=1.6601
#angles['left_w2']=-2.0179
#limb.move_to_joint_positions(angles)

#leftgripper.calibrate(block=True,timeout=5)
leftgripper.open(block=True,timeout=5)
leftgripper.set_velocity(0.01)
leftgripper.close(block=True,timeout=5)




