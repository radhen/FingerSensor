<<<<<<< HEAD
#!/usr/bin/env python
=======
>>>>>>> 93628e441ad2ef0155c460ed7f4fa4d65723e496
from __future__ import division, print_function
import serial
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension

def collect_data():
<<<<<<< HEAD
    with serial.Serial('/dev/ttyACM1', 115200, timeout=0.2) as ser:
            while True:
                ser.write(b'm')
                values = [int(i) for i in ser.readline().strip().split()]
                if len(values) != 16:
                    continue
                yield values
=======
    with serial.Serial('/dev/ttyACM1',115200) as ser:
            try:
                while True:
                    ser.write(b'm')
                    values = [int(i) for i in ser.readline().strip().split()]
                    if len(values) != 16:
                        continue
                    yield values
            except Exception as e:
                print(e)
>>>>>>> 93628e441ad2ef0155c460ed7f4fa4d65723e496


def sensor_node():
    c = collect_data()
    pub = rospy.Publisher('/sensor_values', Int32MultiArray, queue_size=1)
<<<<<<< HEAD
=======
    rospy.init_node('sensor_node')
>>>>>>> 93628e441ad2ef0155c460ed7f4fa4d65723e496
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        values = next(c)
        msg = Int32MultiArray(MultiArrayLayout([MultiArrayDimension('', 16, 1)], 1),
                              values)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
<<<<<<< HEAD
    rospy.init_node('sensor_node')
=======
>>>>>>> 93628e441ad2ef0155c460ed7f4fa4d65723e496
    s = sensor_node()
