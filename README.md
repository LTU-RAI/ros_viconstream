# A C++ ROS library for Vicon

## Contributors

* [Emil Fresk](https://www.github.com/korken89)

---

## License

Licensed under the Boost Software License 1.0, see LICENSE file for details.

---

## Functionality

Publishes geometry messages, that are time-stamped, with the Vicon measurements.

* Supports a "zero-pose" for calibrating an object.
* Supports frame to frame limits on rotation and position to detect if a measurement is invalid.
