import rospy
import baxter_interface
import numpy as np
import serial
import re

rospy.init_node('servo_arm')
limb = baxter_interface.Limb('left')
leftgripper = baxter_interface.Gripper('left')
angles = limb.joint_angles()
 
# Initialize the robot to scanning position
#angles['left_s0']=-0.8015049616701286
#angles['left_s1']=-0.17564080021287987
#angles['left_e0']=0.06902913545484361
#angles['left_e1']=0.2124563391221298
#angles['left_w0']=-1.6041604089311714
#angles['left_w1']=1.6256361399615673
#angles['left_w2']=1.5880536106583745
#speed = limb.set_joint_positions_speed(0.2)
limb.move_to_joint_positions(angles)

# Initialize serial port
ser = serial.Serial('/dev/ttyACM3',115200)
print(ser.name)
while 1:
	ser.write(b'm')
	s=''
	line = ser.readline()
	line = re.sub(' +',',',line)
	line = re.sub('\r\n','',line)
	pre = line.split(",")
	#print pre
	for _i in range(1,16,1):
		#print pre[_i]
		s=s+' '+pre[_i]
	#print s

	dif = [(int(pre[j+8])-int(pre[j])) for j in range(1,8)]
	#print dif
	
	sum1 = 0
	for i in range(0,7,1):
		sum1 = sum1 + dif[i]
	#print sum1 
	avg = sum1/7
	print avg


#angles['left_w1']=2.09
#limb.set_joint_position_speed(0.1)
#limb.move_to_joint_positions(angles,timeout=3.0)
#sleep(0.5)
#angles = limb.joint_angles()
		

