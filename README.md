This ROS package provides some tools facilitating the definition of (static) ROS tf transforms.

First of all, it provides some extensions to rviz:

- An **EulerProperty** class allowing for the definition of arbitrary sets of Euler angles (and conversion between them).
- A **RotationProperty** class combining `EulerProperty` and `QuaternionProperty` to provide flexible means of entering orientation information.
- An **[interactive transform publisher](../../wiki/Display)** as an `rviz::Display` plugin allowing you to define your transforms with an rviz interactive marker.
You can use this also, to interactively perform frame transformations.

Furthermore, it provides two cmdline utilities:

- **static_transform_publisher** is an adapted version of `tf2_ros`'s source code, providing more fine-grained command-line options
to define orientations from an arbitrary set of Euler angles.

- **static_transform_publisher_gui** is an interactive version of the `static_transform_publisher`
allowing you to modify the transform interactively.
