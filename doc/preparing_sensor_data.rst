Preparing Your Data for Use with robot_localization
###################################################

Before getting started with the state estimation nodes in ``robot_localization``, it is important that users ensure that their sensor data well-formed. There are various considerations for each class of sensor data, and users are encouraged to read this tutorial in its entirety before attempting to use ``robot_localization``.

For additional information, users are encouraged to watch this `presentation <https://vimeo.com/142624091>`_ from ROSCon 2015.

Adherence to ROS Standards
**************************

The two most important ROS REPs to consider are:

* `REP-103 <http://www.ros.org/reps/rep-0103.html>`_ (Standard Units of Measure and Coordinate Conventions) 
* `REP-105 <http://www.ros.org/reps/rep-0105.html>`_ (Coordinate Frame Conventions). 

Users who are new to ROS or state estimation are encouraged to read over both REPs, as it will almost certainly aid you in preparing your sensor data. ``robot_localization`` attempts to adhere to these standards as much as possible.

Also, it will likely benefit users to look over the specifications for each of the supported ROS message types:

* `nav_msgs/Odometry <http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html>`_

* `geometry_msgs/PoseWithCovarianceStamped <http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html>`_

* `geometry_msgs/TwistWithCovarianceStamped <http://docs.ros.org/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html>`_

* `sensor_msgs/Imu <http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html>`_

Coordinate Frames and Transforming Sensor Data
**********************************************

`REP-105 <http://www.ros.org/reps/rep-0105.html>`_ specifies four principal coordinate frames: *base_link*, *odom*, *map*, and *earth*. The *base_link* frame is rigidly affixed to the robot. The *map* and *odom* frames are world-fixed frames whose origins are typically aligned with the robot's start position. The *earth* frame is used to provide a common reference frame for multiple *map* frames (e.g., for robots distributed over a large area). The *earth* frame is not relevant to this tutorial.

The state estimation nodes of ``robot_localization`` produce a state estimate whose pose is given in the *map* or *odom* frame and whose velocity is given in the *base_link* frame. All incoming data is transformed into one of these coordinate frames before being fused with the state. The data in each message type is transformed as follows:

* `nav_msgs/Odometry <http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html>`_ - All pose data (position and orientation) is transformed from the message header's ``frame_id`` into the coordinate frame specified by the ``world_frame`` parameter (typically *map* or *odom*). In the message itself, this specifically refers to everything contained within the ``pose`` property. All twist data (linear and angular velocity) is transformed from the ``child_frame_id`` of the message into the coordinate frame specified by the ``base_link_frame`` parameter (typically *base_link*). 
* `geometry_msgs/PoseWithCovarianceStamped <http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html>`_ - Handled in the same fashion as the pose data in the Odometry message.
* `geometry_msgs/TwistWithCovarianceStamped <http://docs.ros.org/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html>`_ - Handled in the same fashion as the twist data in the Odometry message.
* `sensor_msgs/Imu <http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html>`_ - The IMU message is currently subject to some ambiguity, though this is being addressed by the ROS community. Most IMUs natively report orientation data in a world-fixed frame whose :math:`X` and :math:`Z` axes are defined by the vectors pointing to magnetic north and the center of the earth, respectively, with the Y axis facing east (90 degrees offset from the magnetic north vector). This frame is often referred to as NED (North, East, Down). However, `REP-103 <http://www.ros.org/reps/rep-0103.html>`_ specifies an ENU (East, North, Up) coordinate frame for outdoor navigation. As of this writing, ``robot_localization`` assumes an ENU frame for all IMU data, and does not work with NED frame data. This may change in the future, but for now, users should ensure that data is transformed to the ENU frame before using it with any node in ``robot_localization``.
 
The IMU may also be oriented on the robot in a position other than its "neutral" position. For example, the user may mount the IMU on its side, or rotate it so that it faces a direction other than the front of the robot. This offset is typically specified by a static transform from the ``base_link_frame`` parameter to the IMU message's ``frame_id``. The state estimation nodes in ``robot_localization`` will automatically correct for the orientation of the sensor so that its data aligns with the frame specified by the ``base_link_frame`` parameter. 

Handling tf_prefix
******************

With the migration to `tf2 <http://wiki.ros.org/tf2>`_ as of ROS Indigo, ``robot_localization`` still allows for the use of the ``tf_prefix`` parameter, but, in accordance with `tf2 <http://wiki.ros.org/tf2>`_, all ``frame_id`` values will have any leading '/' stripped.

Considerations for Each Sensor Message Type
*******************************************

Odometry
========

Many robot platforms come equipped with wheel encoders that provide instantaneous translational and rotational velocity. Many also internally integrate these velocities to generate a position estimate. If you are responsible for this data, or can edit it, keep the following in mind:

1. **Velocities/Poses:** `robot_localization` can integrate velocities or absolute pose information. In general, the best practice is:

 * If the odometry provides both position and linear velocity, fuse the linear velocity. 
 * If the odometry provides both orientation and angular velocity, fuse the orientation.

 .. note:: If you have two sources of orientation data, then you'll want to be careful. If both produce orientations with accurate covariance matrices, it's safe to fuse the orientations. If, however, one or both under-reports its covariance, then you should only fuse the orientation data from the more accurate sensor. For the other sensor, use the angular velocity (if it's provided), or continue to fuse the absolute orientation data, but turn `_differential` mode on for that sensor. 

2. **frame_id:** See the section on coordinate frames and transforms above.

3. **Covariance:** Covariance values **matter** to ``robot_localization``. `robot_pose_ekf <http://wiki.ros.org/robot_pose_ekf>`_ attempts to fuse all pose variables in an odometry message. Some robots' drivers have been written to accommodate its requirements. This means that if a given sensor does not produce a certain variable (e.g., a robot that doesn't report :math:`Z` position), then the only way to get ``robot_pose_ekf`` to ignore it is to inflate its variance to a very large value (on the order of :math:`1e3`) so that the variable in question is effectively ignored. This practice is both unnecessary and even detrimental to the performance of ``robot_localization``. The exception is the case where you have a second input source measuring the variable in question, in which case inflated covariances will work.

 .. note:: See :ref:`configuring_robot_localization` and :ref:`migrating_from_robot_pose_ekf` for more information.
 
4. **Signs:** Adherence to `REP-103 <http://www.ros.org/reps/rep-0103.html>`_ means that you need to ensure that the **signs** of your data are correct. For example, if you have a ground robot and turn it counter-clockwise, then its yaw angle should *increase*, and its yaw velocity should be *positive*. If you drive it *forward*, its X-position should *increase* and its X-velocity should be *positive*. 

5. **Transforms:** Broadcast of the *odom*->*base_link* transform. When the ``world_frame`` parameter is set to the value of the ``odom_frame`` parameter in the configuration file, ``robot_localization``'s state estimation nodes output both a position estimate in a `nav_msgs/Odometry <http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html>`_ message and a transform from the frame specified by its ``odom_frame`` parameter to its ``base_link_frame`` parameter. However, some robot drivers also broadcast this transform along with their odometry message. If users want ``robot_localization`` to be responsible for this transform, then they need to disable the broadcast of that transform by their robot's driver. This is often exposed as a parameter.

IMU
===

In addition to the following, be sure to read the above section regarding coordinate frames and transforms for IMU data.

1. **Adherence to specifications:** As with odometry, be sure your data adheres to `REP-103 <http://www.ros.org/reps/rep-0103.html>`_ and the `sensor_msgs/Imu <http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html>`_ specification. Double-check the signs of your data, and make sure the ``frame_id`` values are correct.

2. **Covariance:** Echoing the advice for odometry, make sure your covariances make sense. Do not use large values to get the filter to ignore a given variable. Set the configuration for the variable you'd like to ignore to *false*. 

3. **Acceleration:** Be careful with acceleration data. The state estimation nodes in ``robot_localization`` assume that an IMU that is placed in its neutral *right-side-up* position on a flat surface will:

 * Measure **+**:math:`9.81` meters per second squared for the :math:`Z` axis. 
 * If the sensor is rolled **+**:math:`90` degrees (left side up), the acceleration should be **+**:math:`9.81` meters per second squared for the :math:`Y` axis. 
 * If the sensor is pitched **+**:math:`90` degrees (front side down), it should read **-**:math:`9.81` meters per second squared for the :math:`X` axis.

PoseWithCovarianceStamped
=========================

See the section on odometry.

TwistWithCovarianceStamped
==========================

See the section on odometry.

Common errors
*************

* Input data doesn't adhere to `REP-103 <http://www.ros.org/reps/rep-0103.html>`_. Make sure that all values (especially orientation angles) increase and decrease in the correct directions.
* Incorrect ``frame_id`` values. Velocity data should be reported in the frame given by the ``base_link_frame`` parameter, or a transform should exist between the ``frame_id`` of the velocity data and the ``base_link_frame``.
* Inflated covariances. The preferred method for ignoring variables in measurements is through the ``odomN_config`` parameter. 
* Missing covariances. If you have configured a given sensor to fuse a given variable into the state estimation node, then the variance for that value (i.e., the covariance matrix value at position :math:`(i, i)`, where :math:`i` is the index of that variable) should **not** be :math:`0`. If a :math:`0` variance value is encountered for a variable that is being fused, the state estimation nodes will add a small epsilon value (:math:`1e^{-6}`) to that value. A better solution is for users to set covariances appropriately.

