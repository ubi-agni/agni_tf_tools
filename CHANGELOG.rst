^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agni_tf_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2020-04-15)
------------------
* Replace string-based SIGNALs with function pointers
* Always show marker type property
* EulerProperty: use QDoubleSpinBoxes (allowing animated motion)
* Display: Fix several issues with frame updates
  * Always consider return value of fillPoseStamped() and don't publish on failure
  * On failure, register to TF changes to retry later
  * Finish tf2 transition: use tf2::BufferCore instead of tf::Transformer
* Handle multiple TransformBroadcasters in the same ROS node.
* Contributors: Robert Haschke

0.1.4 (2020-03-27)
------------------
* migration to tf2
* modernize code
* Contributors: Robert Haschke

0.1.3 (2020-03-27)
------------------
* remove superfluous marker_node\_
* avoid spurious tf messages when display was saved in disabled state
* Contributors: Robert Haschke

0.1.2 (2019-06-07)
------------------
* fix compiler warnings
* cmake: cleanup, correctly announce exported lib
* Contributors: Robert Haschke

0.1.1 (2018-10-30)
------------------
* install header files for rviz properties
* Contributors: Robert Haschke
