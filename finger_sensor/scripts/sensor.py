#!/usr/bin/env python
from __future__ import division, print_function

import serial
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension


def collect_data():
    with serial.Serial('/dev/ttyACM0', 115200) as ser:
        # Give it some time to initialize
        data = []
        N = 5
        for i in range(N):
            data.append(ser.read(ser.inWaiting()))
            rospy.loginfo("Waiting for {} s more".format(N-i))
            rospy.sleep(1)
        print('\n'.join(filter(None, data)))
        while True:
            ser.write(b'm')
            line = ser.readline()
            try:
                values = [int(i) for i in line.strip().split()]
            except ValueError:
                rospy.logdebug(line)
                continue
            if len(values) == 16:
                yield values


def sensor_node():
    c = collect_data()
    pub = rospy.Publisher('/sensor_values', Int32MultiArray, queue_size=1)
    rospy.init_node('sensor_node')
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        values = next(c)
        msg = Int32MultiArray(
            MultiArrayLayout([MultiArrayDimension('sensor data', 16, 1)], 1),
            values)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('sensor_node')
    s = sensor_node()
