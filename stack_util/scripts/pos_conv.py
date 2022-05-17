#!/usr/bin/env python2
from quadrotor_msgs.msg import PositionCommand
# from mavros_msgs.msg import PositionTarget
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import rospy
from tf.transformations import quaternion_from_euler
# import tf
from geometry_msgs.msg import Transform, Twist

"""
convert the message from fast-planner to the type accepted by the geometric_controller
rewritten but basically copied from
https://github.com/mzahana/px4_fast_planner/blob/master/scripts/trajectory_msg_converter.py
"""


class Conversion():

    def __init__(self):
        rospy.init_node("pos_cmd_2_att_targ")

        self.pos_cmd_sub = rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.pos_cmd_cb, queue_size=10)
        # self.att_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        # this is the topic geometric_controller subscribes to for the trajectory
        self.traj_pub = rospy.Publisher('/command/trajectory', MultiDOFJointTrajectory, queue_size=1)

        rospy.spin()

    def pos_cmd_cb(self, msg):
        """
        the message contains many fields and is composed of other different messages, which are then sewn together
        """

        pose = Transform()
        pose.translation.x = msg.position.x
        pose.translation.y = msg.position.y
        pose.translation.z = msg.position.z
        q = quaternion_from_euler(0, 0, msg.yaw)
        # q = tf.transformations.quaternion_from_euler(0, 0, msg.yaw)

        pose.rotation.x = q[0]
        pose.rotation.y = q[1]
        pose.rotation.z = q[2]
        pose.rotation.w = q[3]

        # velocity
        vel = Twist()
        vel.linear = msg.velocity
        # not sure if the vertical axes are aligned
        # vel.angular.z = msg.yaw_dot

        acc = Twist()
        acc.linear = msg.acceleration

        traj_point = MultiDOFJointTrajectoryPoint()
        traj_point.transforms.append(pose)
        traj_point.velocities.append(vel)
        traj_point.accelerations.append(acc)

        traj_msg = MultiDOFJointTrajectory()

        traj_msg.header = msg.header
        traj_msg.points.append(traj_point)

        # according to the same programme as below it is 1 in uint
        self.traj_pub.publish(traj_msg)


if __name__ == "__main__":
    conv = Conversion()
