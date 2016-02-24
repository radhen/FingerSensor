from __future__ import division, print_function
import numpy as np
import rospy
import baxter_interface
from std_msgs.msg import Int32MultiArray, Float64
from baxter_pykdl import baxter_kinematics
from sensor import sensor_node
from time import time


def error(sensor_values):
    # Sensors 8th and 16th are not used (they point forward)
    diff = [sensor_values[j+8]-sensor_values[j] for j in range(7)]
    err = sum(diff) / len(diff)
    return err


class Controller(object):
    def __init__(self, P=1/640000):
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


    def control(self, msg):
        values = msg.data
        err = error(values)

        y_v = -np.clip(self.P * err, -0.1, 0.1)
        self.err_pub.publish(y_v)
        cartesian_v = [0, y_v, 0, 0, 0, 0]
        joint_v = self.compute_joint_velocities(cartesian_v, self.jinv)
        self.limb.set_joint_velocities(joint_v)

    def compute_joint_velocities(self, cartesian_velocities, jinv=None):
        if jinv is None:
            jinv = baxter_kinematics(self.limb_name).jacobian_pseudo_inverse()
        joint_v = np.squeeze(np.asarray(jinv.dot(cartesian_velocities)))

    	joint_v_dict = {'{}_{}'.format(self.limb_name, joint_name): val for
            joint_name, val in zip(['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2'],
                                    joint_v)}
    	joint_v_dict.update({'{}_{}'.format(self.other_limb_name, joint_name): 0.0 for
            joint_name in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']})
    	joint_v_dict.update({'head_nod':0.0 , 'head_pan':0.0, 'torso_t0':0.0})

        return joint_v_dict

if __name__ == '__main__':
    c = Controller()
    rospy.spin()
