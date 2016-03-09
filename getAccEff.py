import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from scipy import signal
from std_msgs.msg import Int32MultiArray, Float64
import numpy as np
import matplotlib.pyplot as plt

def callback(data):
	acc_x.append(data.linear_acceleration.x)
	acc_y.append(data.linear_acceleration.y)
	acc_z.append(data.linear_acceleration.z)
	#rospy.loginfo("%s X: %s", data.header.stamp.nsecs,data.linear_acceleration.x)
	#rospy.loginfo("%s Y: %s", data.header.stamp.nsecs,data.linear_acceleration.y)
	#rospy.loginfo("%s Z: %s", data.header.stamp.nsecs,data.linear_acceleration.z)

def callback2(data):
	if len(data.effort) == 17:
		eff.append(data.effort[8])
		#rospy.loginfo("%s E: %s",data.header.stamp.nsecs, data.effort[8])

def callback3(msg):
	values.append(msg.data)


if __name__ == "__main__":
	rospy.init_node('AccEff_listener')
	values = []
	acc_x = []
	acc_y = []
	acc_z = []
	eff = []
	sub1=rospy.Subscriber("/robot/accelerometer/left_accelerometer/state", Imu,callback)
 	sub2=rospy.Subscriber("/robot/joint_states", JointState,callback2)
	sub3=rospy.Subscriber('/sensor_values', Int32MultiArray, callback3, queue_size=1)
	rospy.sleep(4)
	sub1.unregister()
	sub2.unregister()
	sub3.unregister()
	values = np.array(values)
	print values.shape

	eff = np.array(eff)
	acc_x = np.array(acc_x)
	acc_y = np.array(acc_y)
	acc_z = np.array(acc_z)
	#print type(eff)

	b1, a1 = signal.butter(1, 0.48, 'high', analog=False)
	f_acc_x = signal.lfilter(b1, a1, acc_x, axis=-1, zi=None)
	f_acc_y = signal.lfilter(b1, a1, acc_y, axis=-1, zi=None)
	f_acc_z = signal.lfilter(b1, a1, acc_z, axis=-1, zi=None)
	eff = signal.lfilter(b1, a1, eff, axis=-1, zi=None)
	FAII = np.sqrt(np.square(f_acc_x) + np.square(f_acc_y) + np.square(f_acc_z))
	#print f_acc_x.size

	#plt.plot(acc_x)
	#plt.plot(f_acc_x)
	#plt.show()
