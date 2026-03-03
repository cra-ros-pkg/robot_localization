Lifecycle Node Support
======================

`robot_localization` nodes (EKF/UKF) support the ROS 2 managed lifecycle. This allows running the node in either an automatic (backward-compatible) mode or in a managed mode where an external controller performs the lifecycle transitions.

Control is via the ``autostart`` launch argument:

- ``true`` (default): The node will configure and activate itself automatically on startup (preserves previous behavior).
- ``false`` : The node will remain in the UNCONFIGURED state and must be transitioned to CONFIGURED/ACTIVE with ``ros2 lifecycle`` commands.

Examples
--------

Launch the EKF node in managed mode:

.. code-block:: bash

   ros2 launch robot_localization ekf.launch.py autostart:=false

Manage the lifecycle of a running node (replace ``/ekf_node`` with your node name if different):

.. code-block:: bash

   ros2 lifecycle set /ekf_node configure
   ros2 lifecycle set /ekf_node activate
   ros2 lifecycle set /ekf_node deactivate
   ros2 lifecycle set /ekf_node cleanup
   ros2 lifecycle set /ekf_node shutdown

Notes
-----

- When running with ``autostart:=false``, the node will not publish filtered output until it has been activated via a lifecycle transition.
- The launch files ``launch/ekf.launch.py`` and ``launch/ukf.launch.py`` expose the ``autostart`` argument.
- Default behavior (``true``) preserves compatibility with existing setups that expect the node to start and publish immediately.
- **Deprecation Note**: Starting in the *Lyrical* release, the default for ``autostart`` will change from ``true`` to ``false``.