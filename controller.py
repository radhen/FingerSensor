<<<<<<< HEAD
#!/usr/bin/env python
from __future__ import division, print_function
from cStringIO import StringIO
import sys

import numpy as np

=======
from __future__ import division, print_function
import numpy as np
>>>>>>> 93628e441ad2ef0155c460ed7f4fa4d65723e496
import rospy
import baxter_interface
from std_msgs.msg import Int32MultiArray, Float64
from baxter_pykdl import baxter_kinematics
<<<<<<< HEAD

from sensor import sensor_node
=======
from sensor import sensor_node
from time import time
>>>>>>> 93628e441ad2ef0155c460ed7f4fa4d65723e496


def error(sensor_values):
    # Sensors 8th and 16th are not used (they point forward)
    diff = [sensor_values[j+8]-sensor_values[j] for j in range(7)]
    err = sum(diff) / len(diff)
    return err


class Controller(object):
    def __init__(self, P=1/640000):
<<<<<<< HEAD
        self.limb_name = 'left'
        self.other_limb_name = 'right'
        self.limb = baxter_interface.Limb(self.limb_name)
        self.err_pub = rospy.Publisher('/error', Float64, queue_size=1)
        self.P = P
        self.kinematics = baxter_kinematics(self.limb_name)
        self.jinv = self.kinematics.jacobian_pseudo_inverse()
        #print(self.jinv)
        #print(self.kinematics.jacobian())

    def enable(self):
        self.sensor_subscriber = rospy.Subscriber('/sensor_values',
                                                 Int32MultiArray,
                                                 self.control,
                                                 queue_size=1)

    def disable(self):
        self.sensor_subscriber.unregister()
=======
        rospy.init_node('baxter_controller')
        self.limb_name = 'left'
        self.other_limb_name = 'right'
        self.limb = baxter_interface.Limb(self.limb_name)
        self.sensor_subscriber = rospy.Subscriber('/sensor_values',
                                                 Int32MultiArray,
                                                 self.control)

        self.err_pub = rospy.Publisher('/error', Float64, queue_size=1)
        self.P = P
        self.jinv = baxter_kinematics(self.limb_name).jacobian_pseudo_inverse()

>>>>>>> 93628e441ad2ef0155c460ed7f4fa4d65723e496

    def control(self, msg):
        values = msg.data
        err = error(values)
<<<<<<< HEAD
        #print(err)
        y_v = -np.clip(self.P * err, -0.1, 0.1)
        #print(y_v)
        self.err_pub.publish(y_v)
        cartesian_v = [0, y_v, 0, 0, 0, 0]
        joint_v = self.compute_joint_velocities(cartesian_v)
        #print(joint_v)
=======

        y_v = -np.clip(self.P * err, -0.1, 0.1)
        self.err_pub.publish(y_v)
        cartesian_v = [0, y_v, 0, 0, 0, 0]
        joint_v = self.compute_joint_velocities(cartesian_v, self.jinv)
>>>>>>> 93628e441ad2ef0155c460ed7f4fa4d65723e496
        self.limb.set_joint_velocities(joint_v)

    def compute_joint_velocities(self, cartesian_velocities, jinv=None):
        if jinv is None:
<<<<<<< HEAD
            self.jinv = self.kinematics.jacobian_pseudo_inverse()

        joint_v = np.squeeze(np.asarray(self.jinv.dot(cartesian_velocities)))
=======
            jinv = baxter_kinematics(self.limb_name).jacobian_pseudo_inverse()
        joint_v = np.squeeze(np.asarray(jinv.dot(cartesian_velocities)))

>>>>>>> 93628e441ad2ef0155c460ed7f4fa4d65723e496
    	joint_v_dict = {'{}_{}'.format(self.limb_name, joint_name): val for
            joint_name, val in zip(['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2'],
                                    joint_v)}
    	joint_v_dict.update({'{}_{}'.format(self.other_limb_name, joint_name): 0.0 for
            joint_name in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']})
    	joint_v_dict.update({'head_nod':0.0 , 'head_pan':0.0, 'torso_t0':0.0})

        return joint_v_dict

if __name__ == '__main__':
<<<<<<< HEAD
    rospy.init_node('baxter_controller')
    c = Controller()
    c.enable()
=======
    c = Controller()
>>>>>>> 93628e441ad2ef0155c460ed7f4fa4d65723e496
    rospy.spin()
