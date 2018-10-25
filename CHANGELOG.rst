^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.5 (2018-10-25)
------------------
* Linting
* Contributors: Tom Moore

2.4.4 (2018-10-25)
------------------
* Adding more output for measurement history failures
* Adding filter processing toggle service
* Waiting for valid ROS time before starting navsat_transform_node
* Contributors: Tom Moore, stevemacenski

2.4.3 (2018-04-11)
------------------
* Add published accel topic to documentation
* Adding log statements for nans in the invertable matrix
* Fixing issue with potential seg fault
* Contributors: Oleg Kalachev, Tom Moore, stevemacenski

2.4.2 (2018-01-03)
------------------
* Fixing CMakeLists
* Contributors: Tom Moore

2.4.1 (2017-12-15)
------------------
* Fixing datum precision
* Fixing state history reversion
* Silencing unnecessary errors and warnings
* Fixing critical bug with dynamic process noise covariance
* Some trivial changes to lessen the differences to lunar
* Fix typo in reading Mahalanobis thresholds.
* Zero out rotation in GPS to base_link transform
* Update xmlrpcpp includes for Indigo support
* Removing lastUpdateTime
* Fixing timestamps in map->odom transform
* Simplify enabledAtStartup logic
* Add std_srvs dependency
* Add enabling service
* Ensure all raw sensor input orientations are normalized even if messages are not
* Install params directory
* Add robot localization estimator
* Contributors: Jacob Perron, Jacob Seibert, Jiri Hubacek, Mike Purvis, Miquel Massot, Pavlo Kolomiiets, Rein Appeldoorn, Rokus Ottervanger, Tom Moore, stevemacenski

2.4.0 (2017-06-12)
------------------
* Updated documentation
* Added reset_on_time_jump option
* Added feature to optionally publish utm frame as parent in navsat_transform_node
* Moved global callback queue reset
* Added initial_state parameter and documentation
* Fixed ac/deceleration gains default logic
* Added gravity parameter
* Added delay and throttle if tf lookup fails
* Fixed UKF IMUTwistBasicIO test
* Added transform_timeout parameter
* Set gps_odom timestamp before tf2 lookuptransform
* Removed non-portable sincos calls
* Simplified logic to account for correlated error
* Added dynamic process noise covariance calculation
* Fixed catkin_package Eigen warning
* Added optional publication of acceleration state
* Contributors: Brian Gerkey, Enrique Fernandez, Jochen Sprickerhof, Rein Appeldoorn, Simon Gene Gottlieb, Tom Moore

2.3.1 (2016-10-27)
------------------
* Adding gitignore
* Adding remaining wiki pages
* Adding config and prep pages
* Adding navsat_transform_node documentation
* use_odometry_yaw fix for n_t_n
* Fixing issue with manual pose reset when history is not empty
* Getting inverse transform when looking up robot's pose.
* Sphinx documentation
* Removing forward slashes from navsat_transform input topics for template launch file
* Adding example launch and parameter files for a two-level EKF setup with navsat_transform_node
* Adding yaml file for navsat_transform_node, and moving parameter documentation to it.
* Updating EKF and UKF parameter templates with usage comments
* Contributors: Tom Moore, asimay

2.3.0 (2016-07-28)
------------------
* Fixed issues with datum usage and frame_ids
* Fixed comment for wait_for_datum
* Fixing issue with non-zero navsat sensor orientation offsets
* Fixing issue with base_link->gps transform wrecking the 'true' UTM position computation
* Using correct covariance for filtered GPS
* Fixed unitialized odometry covariance bug
* Added filter history and measurement queue behavior
* Changing output timestamp to more accurately use the time stamp of the most recently-processed measurement
* Added TcpNoDelay()
* Added parameter to make transform publishing optional
* Fixed differential handling for pose data so that it doesn't care about the message's frame_id
* Updated UKF config and launch
* Added a test case for the timestamp diagnostics
* Added reporting of bad timestamps via diagnostics
* Updated tests to match new method signatures
* Added control term
* Added smoothing capability for delayed measurements
* Making variables in navsat_transform conform to ROS coding standards
* Contributors: Adel Fakih, Ivor Wanders, Marc Essinger, Tobias Tueylue, Tom Moore

2.2.3 (2016-04-24)
------------------
* Cleaning up callback data structure and callbacks and updating doxygen comments in headers
* Removing MessageFilters
* Removing deprecated parameters
* Adding the ability to handle GPS offsets from the vehicle's origin
* Cleaning up navsat_transform.h
* Making variables in navsat_transform conform to ROS coding standards

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

