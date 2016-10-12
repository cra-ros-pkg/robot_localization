.. _migrating_from_robot_pose_ekf:

Migrating from robot_pose_ekf
#############################

Migration from ``robot_pose_ekf`` is fairly straightforward. This page is meant to highlight relevant differences between the packages to facilitate rapid transitions. 

Covariances in Source Messages
==============================

For ``robot_pose_ekf``, a common means of getting the filter to ignore measurements is to give it a massively inflated covariance, often on the order of 10^3. However, the state estimation nodes in ``robot_localization`` allow users to specify *which* variables from the measurement should be fused with the current state. If your sensor reports zero for a given variable and you don't want to fuse that value with your filter, or if the sensor is known to produce poor data for that field, then simply set its ``xxxx_config`` parameter value to false for the variable in question (see the main page for a description of this parameter). 

However, users should take care: sometimes platform constraints create implicit :math:`0` measurements of variables. For example, a differential drive robot that cannot move instantaneously in the :math:`Y` direction can safely fuse a :math:`0` measurement for :math:`\dot{Y}` with a small covariance value.

The ''differential'' Parameter
==============================

By default, ``robot_pose_ekf`` will take a pose measurement at time :math:`t`, determine the difference between it and the measurement at time :math:`t-1`, transform that difference into the current frame, and then integrate that measurement. This cleverly aids in cases where two sensors are measuring the same pose variable: as time progresses, the values reported by each sensor will start to diverge. If the covariances on at least one of these measurements do not grow appopriately, the filter will eventually start to oscillate between the measured values. By carrying out differential integration, this situation is avoided and measurements are always consistent with the current state.

This situation can be avoided using three different methods for the ``robot_localization`` state estimation nodes:

1. If fusing two different sources for the same pose data (e.g., two different sensors measuring :math:`Z` position), ensure that those sources accurately report their covariances. If the two sources begin to diverge, then their covariances should refect the growing error that must be occurring in at least one of them.

2. If available, fuse velocity data instead of pose data. If you have two separate data sources measuring the same variable, fuse the most accurate one as pose data and the other as velocity.

3. As an alternative to (2), if velocity data is unavailable for a given pose measurement, enable the ``_differential`` parameter for one of the sensors. This will cause it to be differentiated and fused as a velocity.



