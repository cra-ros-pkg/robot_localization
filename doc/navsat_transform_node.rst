navsat_transform_node
*********************

``navsat_transform_node`` takes as input a `nav_msgs/Odometry <http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html>`_ message (usually the output of ``ekf_localization_node`` or ``ukf_localization_node``), a `sensor_msgs/Imu <http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html>`_ containing an accurate estimate of your robot's heading, and a `sensor_msgs/NavSatFix <http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html>`_ message containing GPS data. It produces an odometry message in coordinates that are consistent with your robot's world frame. This value can be directly fused into your state estimate.

.. note:: If you fuse the output of this node with any of the state estimation nodes in ``robot_localization``, you should make sure that the ``odomN_differential`` setting is *false* for that input.

Parameters
==========

~frequency
^^^^^^^^^^
The real-valued frequency, in Hz, at which ``navsat_transform_node`` checks for new `sensor_msgs/NavSatFix <http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html>`_ messages, and publishes filtered `sensor_msgs/NavSatFix <http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html>`_ when ``publish_filtered_gps`` is set to *true*.

~delay
^^^^^^
The time, in seconds, to wait before calculating the transform from GPS coordinates to your robot's world frame.

~magnetic_declination_radians
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Enter the magnetic declination for your location. If you don't know it, see `http://www.ngdc.noaa.gov/geomag-web <http://www.ngdc.noaa.gov/geomag-web>`_ (make sure to convert the value to radians). This parameter is needed if your IMU provides its orientation with respect to the magnetic north.

~yaw_offset
^^^^^^^^^^^
Your IMU should read 0 for yaw when facing east. If it doesn't, enter the offset here (desired_value = offset + sensor_raw_value). For example, if your IMU reports 0 when facing north, as most of them do, this parameter would be ``pi/2`` (~1.5707963). This parameter changed in version ``2.2.1``. Previously, ``navsat_transform_node`` assumed that IMUs read 0 when facing north, so yaw_offset was used acordingly.

~zero_altitude
^^^^^^^^^^^^^^
If this is *true*, the `nav_msgs/Odometry <http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html>`_ message produced by this node has its pose Z value set to 0.

~use_local_cartesian
^^^^^^^^^^^^^^^^^^^^
If this is *true*, the node uses a local East-North-Up (ENU) cartesian coordinate system centered at the GPS datum. If *false*, the node uses UTM coordinates. Defaults to *false* (UTM mode). Local cartesian mode is required for earth frame transform support.

~publish_filtered_gps
^^^^^^^^^^^^^^^^^^^^^
If *true*, ``navsat_transform_node`` will also transform your robot's world frame (e.g., *map*) position back to GPS coordinates, and publish a `sensor_msgs/NavSatFix <http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html>`_ message on the ``/gps/filtered`` topic.

~broadcast_cartesian_transform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If this is *true*, ``navsat_transform_node`` will broadcast the transform between the cartesian frame (UTM or local_enu) and the world frame. See Published Transforms below for more information.

.. note:: The legacy parameter name ``broadcast_utm_transform`` is deprecated but still supported for backward compatibility.

~use_odometry_yaw
^^^^^^^^^^^^^^^^^
If *true*, ``navsat_transform_node`` will not get its heading from the IMU data, but from the input odometry message. Users should take care to only set this to true if your odometry message has orientation data specified in an earth-referenced frame, e.g., as produced by a magnetometer. Additionally, if the odometry source is one of the state estimation nodes in ``robot_localization``, the user should have at least one source of absolute orientation data being fed into the node, with the ``_differential`` and ``_relative`` parameters set to *false*.

~wait_for_datum
^^^^^^^^^^^^^^^
If *true*, ``navsat_transform_node`` will wait to get a datum from either:

* The ``datum`` parameter
* The ``set_datum`` service

~broadcast_cartesian_transform_as_parent_frame
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If *true*, ``navsat_transform_node`` will publish the cartesian->world_frame transform instead of the world_frame->cartesian transform.
Note that for the transform to be published ``broadcast_cartesian_transform`` also has to be set to *true*.

.. note:: The legacy parameter name ``broadcast_utm_transform_as_parent_frame`` is deprecated but still supported for backward compatibility.

~transform_timeout
^^^^^^^^^^^^^^^^^^
This parameter specifies how long we would like to wait if a transformation is not available yet. Defaults to 0 if not set. The value 0 means we just get us the latest available (see ``tf2`` implementation) transform.

~earth_frame_id
^^^^^^^^^^^^^^^
The name of the ECEF earth frame for REP-105 compliant coordinate systems. Defaults to ``earth``. This frame represents the WGS-84 Earth-Centered Earth-Fixed coordinate system using ellipsoidal heights only (no geoid corrections).

~broadcast_earth_transform
^^^^^^^^^^^^^^^^^^^^^^^^^^
If *true*, ``navsat_transform_node`` will broadcast transforms that link the earth frame to the existing cartesian/world frames, enabling REP-105 compliant ECEF coordinate system support. The specific transform chain published depends on the ``broadcast_cartesian_transform`` and ``broadcast_cartesian_transform_as_parent_frame`` settings. Defaults to *false*.

.. note:: This parameter is only valid when ``use_local_cartesian`` is *true*. Earth frame transforms are not supported with UTM coordinates. If you attempt to set both ``broadcast_earth_transform: true`` and ``use_local_cartesian: false``, the node will log an error and disable the earth transform functionality.

Subscribed Topics
=================
* ``imu/data`` A `sensor_msgs/Imu <http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html>`_ message with orientation data

* ``odometry/filtered`` A `nav_msgs/Odometry <http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html>`_ message of your robot's current position. This is needed in the event that your first GPS reading comes after your robot has attained some non-zero pose.

* ``gps/fix`` A `sensor_msgs/NavSatFix <http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html>`_ message containing your robot's GPS coordinates

Published Topics
================
* ``odometry/gps`` A `nav_msgs/Odometry <http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html>`_ message containing the GPS coordinates of your robot, transformed into its world coordinate frame. This message can be directly fused into ``robot_localization``'s state estimation nodes.

* ``gps/filtered`` (optional) A `sensor_msgs/NavSatFix <http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html>`_ message containing your robot's world frame position, transformed into GPS coordinates

Published Transforms
====================
* ``world_frame->cartesian`` (optional) - If the ``broadcast_cartesian_transform`` parameter is set to  *true*, ``navsat_transform_node`` calculates a transform from the cartesian frame (UTM or local_enu) to the ``frame_id`` of the input odometry data. By default, the cartesian frame is published as a child of the world frame by using the inverse transform. With use of the ``broadcast_cartesian_transform_as_parent_frame`` parameter, the cartesian frame will be published as a parent of the world frame. This is useful if you have multiple robots within one TF tree.

* ``earth->world_frame/local_enu`` (optional) - If the ``broadcast_earth_transform`` parameter is set to *true*, and ``use_local_cartesian`` is set to *true*, ``navsat_transform_node`` publishes transforms that link the REP-105 compliant earth frame (ECEF WGS-84) to the existing coordinate frames. The child frame can be either the world_frame of local_enu dependent on other configurations. If ``broadcast_cartesian_transform`` is *false* or ``broadcast_cartesian_transform_as_parent_frame`` is *false*, then the child frame is set as the world_frame. Otherwise, the child frame is set to local_enu.


Heights
=======

All ECEF conversions use WGS-84 **ellipsoidal** heights. No geoid corrections are applied.
