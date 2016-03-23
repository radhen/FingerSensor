
#!/usr/bin/env python
from __future__ import division, print_function
import serial
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension

def collect_data():
    with serial.Serial('/dev/ttyACM0', 115200, timeout=0.2) as ser:
            while True:
                ser.write(b'm')
                values = [int(i) for i in ser.readline().strip().split()]
                if len(values) != 16:
                    continue
                yield values
    with serial.Serial('/dev/ttyACM0',115200) as ser:
            try:
                while True:
                    ser.write(b'm')
                    values = [int(i) for i in ser.readline().strip().split()]
                    if len(values) != 16:
                        continue
                    yield values
            except Exception as e:
                print(e)

def sensor_node():
    c = collect_data()
    pub = rospy.Publisher('/sensor_values', Int32MultiArray, queue_size=1)
    rospy.init_node('sensor_node')
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        values = next(c)
        msg = Int32MultiArray(MultiArrayLayout([MultiArrayDimension('', 16, 1)], 1), values)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('sensor_node')
    s = sensor_node()
