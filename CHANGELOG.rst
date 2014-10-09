^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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




