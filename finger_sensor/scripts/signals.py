#! /usr/bin/env python
from __future__ import print_function, division, absolute_import

from collections import deque

import numpy as np
from scipy import signal

import rospy

from std_msgs.msg import Int32MultiArray, Float64
from sensor_msgs.msg import Imu


class FilterSignal(object):
    def __init__(self):
        # TODO hardcoded for left arm
        self.acc_sub = rospy.Subscriber(
            '/robot/accelerometer/left_accelerometer/state',
            Imu,
            self.handle_acc)

        self.sensor_sub = rospy.Subscriber(
            '/sensor_values',
            Int32MultiArray,
            self.handle_sensor)

        self.sair_pub = rospy.Publisher(
            '/finger_sensor/sair',
            Float64,
            queue_size=5)
        self.sail_pub = rospy.Publisher(
            '/finger_sensor/sail',
            Float64,
            queue_size=5)
        self.fai_pub = rospy.Publisher(
            '/finger_sensor/fai',
            Float64,
            queue_size=5)
        self.faii_pub = rospy.Publisher(
            '/finger_sensor/faii',
            Float64,
            queue_size=5)

        # Every queue should hold about 4 seconds of data
        self.sensor_t = deque(maxlen=80)
        self.sensor_values = deque(maxlen=80)

        self.acc_t = deque(maxlen=400)
        self.acc = deque(maxlen=400)

        # 0.55pi rad/sample. TODO Double check arguments
        self.b1, self.a1 = signal.butter(1, 0.55, 'high', analog=False)
        # 0.48pi rad/sample
        self.b, self.a = signal.butter(1, 0.48, 'high', analog=False)

    def handle_acc(self, msg):
        # Check header of Imu msg
        t = msg.header.stamp.secs
        acc = (msg.linear_acceleration.x,
               msg.linear_acceleration.y,
               msg.linear_acceleration.z)
        # TODO check how uniform t is, if not investigate correct
        # filtering option
        self.acc_t.append(t)
        self.acc.append(acc)

    def handle_sensor(self, msg):
        # TODO maybe time stamp sensor values with header
        # TODO rospy.get_rostime() vs rospy.Time.now()?
        # print(rospy.get_rostime(), rospy.Time.now())  # They're different
        self.sensor_t.append(rospy.Time.now())
        self.sensor_values.append(msg.data)

    def compute_sai(self):
        # Is this right? Adding only 6 values at first, expected 7,
        # all but the front tip...
        right = sum(self.sensor_values[-1][9:15])
        left = sum(self.sensor_values[-1][0:7])
        self.sair_pub.publish(Float64(right))
        self.sail_pub.publish(Float64(left))

    def compute_fai(self):
        # TODO: Begins by substracting the minimum ever seen, maybe we
        # "zero" the sensor instead? But with a highpass filter, it
        # won't matter anyway...
        filtered_values = signal.lfilter(self.b, self.a,
                                         self.sensor_values, axis=0)
        self.fai = filtered_values.sum(axis=1)
        self.fai_pub.publish(Float64(self.fai[-1]))

    def compute_faii(self):
        filtered_acc = signal.lfilter(self.b1, self.a1, self.acc, axis=0)
        self.faii = np.sqrt((filtered_acc**2).sum(axis=1))
        # Publish just the last value
        self.faii_pub.publish(Float64(self.faii[-1]))


if __name__ == '__main__':
    rospy.init_node('filter_signals')
    f = FilterSignal()
    # Give it some time to collect some data
    rospy.sleep(3)
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        f.compute_sai()
        f.compute_fai()
        f.compute_faii()
        r.sleep()
