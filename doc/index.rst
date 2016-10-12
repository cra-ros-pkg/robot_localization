.. _index:

robot_localization wiki
***********************

``robot_localization`` is a collection of state estimation nodes, each of which is an implementation of a nonlinear state estimator for robots moving in 3D space. It contains two state estimation nodes, ``ekf_localization_node`` and ``ukf_localization_node``. In addition, ``robot_localization`` provides ``navsat_transform_node``, which aids in the integration of GPS data.

.. toctree::
   :hidden:

   state_estimation_nodes
   navsat_transform_node
   preparing_sensor_data
   configuring_robot_localization
   migrating_from_robot_pose_ekf
   integrating_gps
   CHANGELOG

Features
========

All the state estimation nodes in ``robot_localization`` share common features, namely:

* Fusion of an arbitrary number of sensors. The nodes do not restrict the number of input sources. If, for example, your robot has multiple IMUs or multiple sources of odometry information, the state estimation nodes within ``robot_localization`` can support all of them.
* Support for multiple ROS message types. All state estimation nodes in ``robot_localization`` can take in `nav_msgs/Odometry <http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html>`_, `sensor_msgs/Imu <http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html>`_, `geometry_msgs/PoseWithCovarianceStamped <http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html>`_, or `geometry_msgs/TwistWithCovarianceStamped <http://docs.ros.org/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html>`_ messages.
* Per-sensor input customization. If a given sensor message contains data that you don't want to include in your state estimate, the state estimation nodes in ``robot_localization`` allow you to exclude that data on a per-sensor basis.
* Continuous estimation. Each state estimation node in ``robot_localization`` begins estimating the vehicle's state as soon as it receives a single measurement. If there is a holiday in the sensor data (i.e., a long period in which no data is received), the filter will continue to estimate the robot's state via an internal motion model.

All state estimation nodes track the 15-dimensional state of the vehicle: :math:`(X, Y, Z, roll, pitch, yaw, \dot{X}, \dot{Y}, \dot{Z}, \dot{roll}, \dot{pitch}, \dot{yaw}, \ddot{X}, \ddot{Y}, \ddot{Z})`.

Other Resources
===============

If you're new to ``robot_localization``, check out the `2015 ROSCon talk <https://vimeo.com/142624091>`_ for some pointers on getting started.

Further details can be found in :download:`this paper <robot_localization_ias13_revised.pdf>`:

.. code-block:: none

  @inproceedings{MooreStouchKeneralizedEkf2014,
    author    = {T. Moore and D. Stouch},
    title     = {A Generalized Extended Kalman Filter Implementation for the Robot Operating System},
    year      = {2014},
    month     = {July},
    booktitle = {Proceedings of the 13th International Conference on Intelligent Autonomous Systems (IAS-13)},
    publisher = {Springer}
  }

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`

