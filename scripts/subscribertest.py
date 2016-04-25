import rospy
from std_msgs.msg import Int32MultiArray, Float64
from sensor import sensor_node

def callback2(data):
	values.append(msg.data)

def listner():
	rospy.Subscriber('/sensor_values', Int32MultiArray, callback2, queue_size=1)
	#rospy.spin()

if __name__ == '__main__':
	rospy.init_node('listner')
	values= []
	listner()
	rospy.sleep(2)
	print values
