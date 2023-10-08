# Known issues with simulating c300 in Isaac

- Currently the wheel friction is set to a value of `10.0` but for rubber it should only require around `0.8`. It was increased to make the odometry match what is observed on the real robot with concrete floors.

- In the omnigraph for c300 a negative time offset is required for the LIDAR messages published by Isaac.
This is because the data's timestamp is ahead of all values available in the tf buffer.

Isaac (and Gazebo) require a `wheel radius multiplier` of less than 1.0 to get the odometry to report correctly.
When navigating the odometry is still not perfect and the localization system needs to compensate more than expected when the base it rotating.

Running each ros2_control hardware interface as if it were real hardware causes the joint commands to come in separately.
This causes jerky execution and can cause the simulation to go unstable.

I am keeping 3 usd files for the arm; as imported, manually tuned gains, and inverting joint angles to make rotation directions match values reported on ROS topic.
Inverting the joint angles looks to be an Isaac bug because the values reported in the UI do not match.
