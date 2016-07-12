#! /usr/bin/env python
from __future__ import division, print_function, absolute_import

import numpy as np

import rospy
import tf
from std_msgs.msg import Header, Int32MultiArray
from geometry_msgs.msg import (Point, Quaternion, Pose, PoseStamped,
                               Vector3, Vector3Stamped)

from pap.robot import Baxter


class SafetyStop(object):
    """In theory, if this node is running, no matter what happens, if
    something gets too close to the sensor tip, the robot will stop
    moving until the obstacle is cleared.

    """
    def __init__(self, topic='/sensor_values'):
        self.bx = Baxter('left')
        self.joint_v = {name: 0.0 for name in self.bx.limb.joint_names()}
        self.stop = False
        self.nh = rospy.init_node('SafetyStop')
        self.sensor_sub = rospy.Subscriber(topic,
                                           Int32MultiArray,
                                           self.udpate_joint_v,
                                           queue_size=1)

    def _cartesian_v_from_sensor_values(self, values):
        tip = values[[7, 15]]
        if np.any(tip < 10**4):
            self.stop = True
        else:
            self.stop = False
        return [0] * 6

    def update_joint_v(self, msg):
        arr = np.array(msg.data)
        cartesian_v = self._cartesian_v_from_sensor_values(arr)
        self.joint_v = self.bx.compute_joint_velocities(cartesian_v)


class ControlArmThroughHand(object):
    def __init__(self, topic='/sensor_values'):
        self.bx = Baxter('left')
        self.values = np.ones(16)

        self.sensor_sub = rospy.Subscriber(topic,
                                           Int32MultiArray,
                                           self.update_sensor_values,
                                           queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.tl = tf.TransformListener()

    def control_from_sensor_values(self):
        log_values = np.log(self.values)
        tip = log_values[[7, 15]]
        # Match which side is which. Ideally, if the sign of the diff
        # matches whether the gripper needs to move towards the
        # positive or negative part of the y axis in left_gripper.
        # That is, we need left side - right side (using left/right
        # like in l_gripper_{l, r}_finger_tip tfs)
        # TODO: might want to take log(values) for a better behaved controller
        inside_diff = log_values[8:15] - log_values[:7]
        scalar_diff = sum(inside_diff) / len(inside_diff)

        # Take negative for one of the sides, so that angles should
        # match for a parallel object in the gripper
        l_angle, _ = np.polyfit(np.arange(7), log_values[:7], 1)
        r_angle, _ = np.polyfit(np.arange(7), -log_values[8:15], 1)
        rospy.loginfo('Angle computed from l: {}'.format(np.rad2deg(l_angle)))
        rospy.loginfo('Angle computed from r: {}'.format(np.rad2deg(r_angle)))
        avg_angle = np.arctan((l_angle + r_angle) / 2.0)

        # Let's get a frame by the middle of the end effector
        # p = Point(0, 0, -0.05)
        p = (0, 0, -0.05)
        # Of course, tf uses (w, x, y, z), Quaternion uses x, y, z,
        # w. However, tf.TransformBroadcaster().sendTransform uses the
        # tf order.
        # q = Quaternion(q[1], q[2], q[3], q[0])
        q = tf.transformations.quaternion_about_axis(avg_angle, (1, 0, 0))
        # We had a lot of trouble sending a transform (p, q) from
        # left_gripper, and then asking for a point in the last frame
        # in the tf coordinate. Timing issues that I couldn't
        # solve. Instead, do it manually here:
        m1 = tf.transformations.quaternion_matrix(q)
        m1[:3, 3] = p
        p = (0, scalar_diff / 100, 0.05)
        m2 = np.eye(4)
        m2[:3, 3] = p
        m = m2.dot(m1)
        # Extract pose now
        p = Point(*m[:3, 3])
        q = Quaternion(*tf.transformations.quaternion_from_matrix(m))
        time = rospy.Time(0)
        h = Header()
        h.frame_id = 'left_gripper'
        h.stamp = time
        pose = Pose(p, q)
        new_endpose = self.tl.transformPose('base', PoseStamped(h, pose))
        self.bx.move_ik(new_endpose)

    def update_sensor_values(self, msg):
        self.values = np.array(msg.data)


class VelocityControlArmThroughHand(object):
    def __init__(self, topic='/sensor_values'):
        self.bx = Baxter('left')
        self.values = np.ones(16)

        self.sensor_sub = rospy.Subscriber(topic,
                                           Int32MultiArray,
                                           self.update_sensor_values,
                                           queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.tl = tf.TransformListener()

    def _transform_rotation_speed(self, v, to='base'):
        h = Header()
        h.stamp = rospy.Time(0)
        h.frame_id = '{}_gripper'.format(self.bx.limb_name)
        v = Vector3(*v)
        v_base = self.tl.transformVector3(to,
                                          Vector3Stamped(h, v)).vector
        v_cartesian = [0, 0, 0, v_base.x, v_base.y, v_base.z]
        return v_cartesian

    def control_from_sensor_values(self):
        log_values = np.log(self.values)
        tip = log_values[[7, 15]]
        # Match which side is which. Ideally, if the sign of the diff
        # matches whether the gripper needs to move towards the
        # positive or negative part of the y axis in left_gripper.
        # That is, we need left side - right side (using left/right
        # like in l_gripper_{l, r}_finger_tip tfs)
        # TODO: might want to take log(values) for a better behaved controller
        inside_diff = log_values[8:15] - log_values[:7]
        scalar_diff = sum(inside_diff) / len(inside_diff)

        # Take negative for one of the sides, so that angles should
        # match for a parallel object in the gripper
        l_angle, _ = np.polyfit(np.arange(7), log_values[:7], 1)
        r_angle, _ = np.polyfit(np.arange(7), -log_values[8:15], 1)
        rospy.loginfo('Angle computed from l: {}'.format(np.rad2deg(l_angle)))
        rospy.loginfo('Angle computed from r: {}'.format(np.rad2deg(r_angle)))
        avg_angle = np.arctan((l_angle + r_angle) / 2.0)
        P = 1
        rotation_speed = (P * avg_angle, 0, 0)
        v_cartesian = self._transform_rotation_speed(rotation_speed)
        v_joint = self.bx.compute_joint_velocities(v_cartesian)
        self.bx.limb.set_joint_velocities(v_joint)

    def update_sensor_values(self, msg):
        self.values = np.array(msg.data)


if __name__ == '__main__':
    mode = 2
    if mode == 0:
        rospy.init_node('SafetyStop')
        s = SafetyStop()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if s.stop:
                s.bx.limb.set_joint_velocities(s.joint_v)
            r.sleep()
    else:
        nh = rospy.init_node('ArmController')
        if mode == 1:
            s = ControlArmThroughHand()
        elif mode == 2:
            s = VelocityControlArmThroughHand()
        rospy.sleep(1)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            s.control_from_sensor_values()
            r.sleep()
