# A C++ ROS library for Vicon

## Contributors

* [Emil Fresk](https://www.github.com/korken89)

---

## License

Licensed under the Boost Software License 1.0, see LICENSE file for details.

---

## Functionality

Publishes Vicon measurements under the following:

* `geometry_msgs::TransformStamped`
* `nav_msgs::Odometry` (also includes linear and angular velocities)
* `tf::StampedTransform`

Has the following per-object settings:

* Zero-pose for calibrating the center and rotation of an object.
* Frame to frame limits on rotation and position to detect if a measurement is invalid. Can happen when a Vicon object is detected on another object, or if the object flips.
* Outputs odometry where the linear and angular velocities frame are selectable.
