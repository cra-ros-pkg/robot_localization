^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.2 (2016-02-04)
------------------
* Updating trig functions to use sincos for efficiency
* Updating licensing information and adding Eigen MPL-only flag
* Added state to imu frame transformation
* Using state orientation if imu orientation is missing
* Manually adding second spin for odometry and IMU data that is passed to message filters
* Reducing delay between measurement reception and filter output
* Zero altitute in intital transform too, when zero altitude param is set
* Fixing regression with conversion back to GPS coordinates
* Switched cropping of orientation data in inovationSubset with mahalanobis check to prevent excluding measurements with orientations bigger/smaller than ± PI
* Fix Jacobian for EKF.
* Removing warning about orientation variables when only their velocities are measured
* Checking for -1 in IMU covariances and ignoring relevant message data
* roslint and catkin_lint applied
* Adding base_link to datum specification, and fixing bug with order of measurement handling when a datum is specified. Also added check to make sure IMU data is transformable before using it.
* Contributors: Adnan Ademovic, Jit Ray Chowdhury, Philipp Tscholl, Tom Moore, ayrton04, kphil

2.2.1 (2015-05-27)
------------------
* Fixed handling of IMU data w.r.t. differential mode and relative mode

2.2.0 (2015-05-22)
------------------
* Added tf2-friendly tf_prefix appending
* Corrected for IMU orientation in navsat_transform
* Fixed issue with out-of-order measurements and pose resets
* Nodes now assume ENU standard for yaw data
* Removed gps_common dependency
* Adding option to navsat_transform_node that enables the use of the heading from the odometry message instead of an IMU.
* Changed frame_id used in setPoseCallback to be the world_frame
* Optimized Eigen arithmetic for signficiant performance boost
* Migrated to tf2
* Code refactoring and reorganization
* Removed roll and pitch from navsat_transform calculations
* Fixed transform for IMU data to better support mounting IMUs in non-standard orientations
* Added feature to navsat_transform_node whereby filtered odometry data can be coverted back into navsat data
* Added a parameter to allow future dating the world_frame->base_link_frame transform.
* Removed deprecated differential setting handler
* Added relative mode
* Updated and improved tests
* Fixing source frame_id in pose data handling
* Added initial covariance parameter
* Fixed bug in covariance copyinh
* Added parameters for topic queue sizes
* Improved motion model's handling of angular velocities when robot has non-zero roll and pitch
* Changed the way differential measurements are handled
* Added diagnostics

2.1.7 (2015-01-05)
------------------
* Added some checks to eliminate unnecessary callbacks
* Updated launch file templates
* Added measurement outlier rejection
* Added failure callbacks for tf message filters
* Added optional broadcast of world_frame->utm transform for navsat_transform_node
* Bug fixes for differential mode and handling of Z acceleration in 2D mode

2.1.6 (2014-11-06)
------------------
* Added unscented Kalman filter (UKF) localization node
* Fixed map->odom tf calculation
* Acceleration data from IMUs is now used in computing the state estimate
* Added 2D mode

2.1.5 (2014-10-07)
------------------
* Changed initial estimate error covariance to be much smaller
* Fixed some debug output
* Added test suite
* Better compliance with REP-105
* Fixed differential measurement handling
* Implemented message filters
* Added navsat_transform_node

2.1.4 (2014-08-22)
------------------
* Adding utm_transform_node to install targets

2.1.3 (2014-06-22)
------------------
* Some changes to ease GPS integration
* Addition of differential integration of pose data
* Some documentation cleanup
* Added UTM transform node and launch file
* Bug fixes

2.1.2 (2014-04-11)
------------------
* Updated covariance correction formulation to "Joseph form" to improve filter stability.
* Implemented new versioning scheme.

2.1.1 (2014-04-11)
------------------
* Added cmake_modules dependency for Eigen support, and added include to silence boost::signals warning from tf include

