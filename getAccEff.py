import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState


def startPrinting():

	def callback(data):
		rospy.loginfo("%s X: %s", data.header.stamp.nsecs,data.linear_acceleration.x)
		rospy.loginfo("%s Y: %s", data.header.stamp.nsecs,data.linear_acceleration.y)
		rospy.loginfo("%s Z: %s", data.header.stamp.nsecs,data.linear_acceleration.z)

	def callback2(data):
		if len(data.effort) == 17:
			rospy.loginfo("%s E: %s",data.header.stamp.nsecs, data.effort[8] )

	rospy.init_node('AccEff_listener')
	rospy.Subscriber("/robot/accelerometer/left_accelerometer/state", Imu, callback)
	rospy.Subscriber("/robot/joint_states", JointState, callback2)


if __name__ == "__main__":
	a = startPrinting()
	rospy.spin()
