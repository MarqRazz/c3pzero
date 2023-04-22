from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from math import cos, sin


class DiffDriveOdom:
    def __init__(self, clock, separation, radius):
        self._clock = clock
        self._frame_id = "odom"
        self._child_frame_id = "base_id"
        self._separation = separation
        self._radius = radius
        self._last_position = (0, 0)
        self._last_time = self._clock.now()
        self._robot_pose = (0, 0, 0)
        self._first = True

    def _get_diff(self, position):
        if self._first:
            self._first = False
            self._last_position = position
        diff = (
            position[0] - self._last_position[0],
            position[1] - self._last_position[1],
        )
        self._last_position = position
        return diff

    def step(self, position, velocity):
        # position is radians tuple (l, r)
        # velocity is m/s tuple (l, r)
        now = self._clock.now()
        time_step = now - self._last_time
        wheel_l, wheel_r = self._get_diff(position)
        delta_s = self._radius * (wheel_r + wheel_l) / 2.0
        theta = self._radius * (wheel_r - wheel_l) / self._separation
        self._robot_pose = (
            self._robot_pose[0] + delta_s * cos(self._robot_pose[2] + (theta / 2.0)),
            self._robot_pose[1] + delta_s * sin(self._robot_pose[2] + (theta / 2.0)),
            self._robot_pose[2] + theta,
        )
        q = quaternion_from_euler(0.0, 0.0, self._robot_pose[2])

        self._last_time = now

        msg = Odometry()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = now.to_msg()
        msg.child_frame_id = self._child_frame_id
        msg.pose.pose.position.x = self._robot_pose[0]
        msg.pose.pose.position.y = self._robot_pose[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = delta_s / (time_step.nanoseconds * 1e9)
        msg.twist.twist.angular.z = theta / (time_step.nanoseconds * 1e9)

        return msg
