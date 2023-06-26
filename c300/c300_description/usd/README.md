# Known issues with simulating c300 in Isaac

- Currently the wheel friction is set to a value of `10.0` but for rubber it should only require around `0.8`. It was increased to make the odometry match what is observed on the real robot with concrete floors.

- In the omnigraph for c300 a negative time offset is required for the LIDAR messages published by Isaac.
This is because the data's timestamp is ahead of all values available in the tf buffer.

Isaac (and Gazebo) require a `wheel radius multiplier` of less than 1.0 to get the odometry to report correctly.
When navigating the odometry is still not perfect and the localization system needs to compensate more than expected when the base it rotating.
