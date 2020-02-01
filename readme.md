ABOUT
==================================
This repository contains the code for the "Elevation offset-based filter" method for pose estimation. It is was written by Andre Przewodowski, Alberto Hiroshi, Tiago Cesar and Fernando Santos Osorio. The authors are from the Laboratory of Mobile Robotics (LRM, in portuguese) of the University of Sao Paulo (USP).

ROS
==================================
The node `filter_node` (package name `elevation_offset_based_filter`) filters the robot state and publishes the corrected `tf` of the robot. 

## Parameters
* `~nparticles` (Default 1000). The number of particles the filter samples.
* `~direct_resample` (Default `false`). If the method resamples right after updating weights (`true`) or if it waits a fixed period of time (`false`).
* `~publish_tf` (Default `false`). Whether to publish the `tf` or not.
* `~frame_id` (Default "base_link"). The frame of the robot.
* `~sub_odom_topic` (Default "odom"). The topic from which the filter receives the robot offsets.
* `~sub_imu_topic` (Default "/sensor/imu/data"). The topic from which the filter receives the orientation.
* `~sub_altitude_topic` (Default "sensor/altitude/data"). The topic that provides altitude offset readings.
* `~map_path`. The file that contains the elevation map. It must be written as follows:
    * The first line contains `<nrow>,<ncol>`, where `<nrow>` and `<ncol>` are the number of rows and columns of the image;
    * The second line contains the range of the W axis (in UTM), e.g. "212000,232000".
    * The third line contains the range of the N coordinates (in UTM), e.g. "7552000,7596000".
    * The fourth line contains the array of altitudes (in meters), *flattened*, of size `<nrow> * <ncol>` and values separated by commas (no spaces).
* `~use_message_covariance` (Default `true`). Whether to use the covariance of sensor messages for weight updates or not.
* `~stdev_odom_x` (Only required if `~use_message_covariance` is `false`): the standard deviation of the odometry in the `x` direction of the robot frame;
* `~stdev_odom_y` (Only required if `~use_message_covariance` is `false`): the standard deviation of the odometry in the `y` direction of the robot frame;
* `~stdev_orientation` (Only required if `~use_message_covariance` is `false`): the standard deviation of the imu messages;
* `~stdev_meas_altitude` (Only required if `~use_message_covariance` is `false`): the standard deviation of the altitude offset measured.

## Subscribed Topics
* `odom` (Default). *Type* `nav_msgs/Odometry`.
* `sensor/altitude/data` (Default). *Type* `nav_msgs/Odometry`.
* `sensor/imu/data` (Default). *Type* `nav_msgs/Imu`.

## Published Topics
* `filter/particles`. *Type* `sensor_msgs/PointCloud2`. The particles of the filter.
* `filter/pose`. *Type* `geometry_msgs/PoseWithCovarianceStamped`. The estimated pose of the robot.
* `filter/map`. *Type* `sensor_msgs/Image`. The position of the particles in the map.

TODO
==================================
- [ ] Implement odometry TF's and coordinate frame change inside the filter node, not assume received by the odometry nodes.