# -*- coding: utf-8 -*-
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from . import roboclaw_3
from . import diff_drive_odom

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry

from std_msgs.msg import Header

import math

from pprint import pprint


def euler_from_quaternion(odom_msg):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x = odom_msg.pose.pose.orientation.x
    y = odom_msg.pose.pose.orientation.y
    z = odom_msg.pose.pose.orientation.z
    w = odom_msg.pose.pose.orientation.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


class RoboclawTwistSubscriber(Node):
    def __init__(self):
        super().__init__("roboclaw_twist_subscriber")

        cmd_vel_topic = "c3pzero/cmd_vel"
        self.wheel_radius = 0.1715  # meters
        self.wheel_circumference = 2 * math.pi * self.wheel_radius  # meters
        self.ppr = 11600  # pulses per wheel revolution
        self.wheel_track = 0.54  # y distance between left and righ wheel

        self.subscription = self.create_subscription(
            Twist, cmd_vel_topic, self.twist_listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.odom_publisher_ = self.create_publisher(Odometry, "c3pzero/odometry", 10)
        self.br = TransformBroadcaster(self)
        self.joint_publisher_ = self.create_publisher(JointState, "joint_states", 10)
        try:
            self.rc = roboclaw_3.Roboclaw("/dev/ttyACM0", 115200)
        except:
            self.get_logger().error("failed to open port")
            raise
        self.rc_address = 0x80

        version = self.rc.ReadVersion(self.rc_address)
        if version[0] == False:
            self.get_logger().error("Retrieving the Roboclaw version failed")
        else:
            self.get_logger().info("Roboclaw version: %s" % repr(version[1]))

        self.rc.ResetEncoders(self.rc_address)

        self.diff_drive_odom = diff_drive_odom.DiffDriveOdom(
            self.get_clock(), self.wheel_track, self.wheel_radius
        )
        self.create_timer(0.02, self.odom_callback)

        self.get_logger().info(
            "Init complete, listening for twist commands on topic: %s" % cmd_vel_topic
        )

    def twist_listener_callback(self, msg):
        # self.get_logger().info('X_vel: %f, Z_rot: %f' % (0.4*msg.linear.x, msg.angular.z))

        right_wheel = msg.linear.x + (msg.angular.z * self.wheel_track) / 2  # meters / sec
        left_wheel = msg.linear.x - (msg.angular.z * self.wheel_track) / 2 # meters / sec

        wheel_cmds = self.mps_to_pps((right_wheel, left_wheel))
        self.rc.SpeedM1(self.rc_address, wheel_cmds[0])
        self.rc.SpeedM2(self.rc_address, wheel_cmds[1])

    def odom_callback(self):
        """
        the roboclaw returns the encoder position and velocity in a tuple
        the first value is if the read was successful
        the second value is the result (position pulses or rate)
        the third value is ???
        """

        right_wheel_enc = self.rc.ReadEncM1(self.rc_address)
        left_wheel_enc = self.rc.ReadEncM2(self.rc_address)
        # if reading the wheel velocities was unsuccessful return
        if right_wheel_enc[0] == 0 | right_wheel_enc[0] == 0:
            self.get_logger().error("Failed retrieving the Roboclaw wheel positions")
            return

        right_wheel_pps = self.rc.ReadSpeedM1(self.rc_address)  # pulses per second.
        left_wheel_pps = self.rc.ReadSpeedM2(self.rc_address)
        # if reading the wheel velocities was unsuccessful return
        if right_wheel_pps[0] == 0 | left_wheel_pps[0] == 0:
            self.get_logger().error("Failed retrieving the Roboclaw wheel velocities")
            return

        # convert the wheel positions to radians
        wheel_pos = self.enc_to_rad((right_wheel_enc[1], left_wheel_enc[1]))
        # convert the wheel speeds to meters / sec
        wheel_speed = self.pps_to_mps((right_wheel_pps[1], left_wheel_pps[1]))

        odom_msg = self.diff_drive_odom.step(wheel_pos, wheel_speed)
        # pprint(odom_msg.pose.pose.position)

        self.get_logger().debug(
            "Pose: x=%f, y=%f theta=%f"
            % (
                odom_msg.pose.pose.position.x,
                odom_msg.pose.pose.position.y,
                euler_from_quaternion(odom_msg)[2],
            )
        )
        self.odom_publisher_.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_msg.pose.pose.orientation.x
        t.transform.rotation.y = odom_msg.pose.pose.orientation.y
        t.transform.rotation.z = odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = odom_msg.pose.pose.orientation.w

        # Send the transformation
        self.br.sendTransform(t)

        wheel_state = JointState()
        wheel_state.header.stamp = self.get_clock().now().to_msg()
        wheel_state.name = ['drivewhl_r_joint', 'drivewhl_l_joint']
        wheel_state.position = wheel_pos
        wheel_state.velocity = wheel_speed
        wheel_state.effort = []
        self.joint_publisher_.publish(wheel_state)

    def mps_to_pps(self, wheel_speed):
        right_wheel_pluses = int(wheel_speed[0] / self.wheel_circumference * self.ppr)
        left_wheel_pluses = int(wheel_speed[1] / self.wheel_circumference * self.ppr)
        return (right_wheel_pluses, left_wheel_pluses)

    def enc_to_rad(self, wheel_pulses):
        right_wheel_pos = wheel_pulses[0] / self.ppr * 2 * math.pi
        left_wheel_pos = wheel_pulses[1] / self.ppr * 2 * math.pi
        # self.get_logger().info('right=%f, left=%f' % (right_wheel_pos, left_wheel_pos))
        return (right_wheel_pos, left_wheel_pos)

    def pps_to_mps(self, wheel_pulses_per_sec):
        right_wheel_speed = (
            wheel_pulses_per_sec[0] / self.ppr * self.wheel_circumference
        )
        left_wheel_speed = wheel_pulses_per_sec[1] / self.ppr * self.wheel_circumference
        return (right_wheel_speed, left_wheel_speed)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = RoboclawTwistSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
