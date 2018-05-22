//          Copyright Emil Fresk 2015-2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "ros_viconstream/ros_viconstream.h"

/*********************************
 * Private members
 ********************************/

ros_viconstream::ObjectPublisher::ObjectPublisher(
    ros::NodeHandle &nh, const std::string &objectPrefix,
    const std::string &subjectName, const std::string &segmentName,
    const double framerate_hz)
    : calibrated(false)
    , global_frame_vel(false)
    , global_frame_rot(false)
    , frame_dt(1.0 / framerate_hz)
    , first_frame(true)
    , zero_pose(tf::Pose::getIdentity())
{
  std::string params;

  double px, py, pz, qw, qx, qy, qz, rr, rp, ry;
  double max_drot, max_dpos;
  bool cal = true;
  bool has_quaternion;
  occluded_counter = 0;

  if (framerate_hz == 0)
		frame_dt = 1; // Some safe number

  /* Create object name. */
  if (objectPrefix.size() > 0)
    name = objectPrefix + "/" + subjectName + "/" + segmentName;
  else
    name = subjectName + "/" + segmentName;

  ROS_INFO("New object detected, adding %s", name.c_str());

  /* Advertise the data messages. */
  pub      = nh.advertise< geometry_msgs::TransformStamped >(name, 1);
  pub_odom = nh.advertise< nav_msgs::Odometry >(name + "/odom", 1);

  /* Advertise the error message. */
  pub_error = nh.advertise< std_msgs::String >(name + "/errors", 1);

  /* Odometry frame calibration. */
  if (!nh.getParam(name + "/global_frame_linear_velocity", global_frame_vel))
  {
    ROS_WARN("Linear velocity frame not selected, defaulting to local frame");
    global_frame_vel = false;
  }
  else
  {
    if (global_frame_vel)
      ROS_INFO_STREAM("Linear velocity frame for " << name
                                                   << " is global frame.");
    else
      ROS_INFO_STREAM("Linear velocity frame for " << name
                                                   << " is local frame.");
  }

  if (!nh.getParam(name + "/global_frame_angular_velocity", global_frame_rot))
  {
    ROS_WARN("Angular velocity frame not selected, defaulting to local frame");
    global_frame_rot = false;
  }
  else
  {
    if (global_frame_rot)
      ROS_INFO_STREAM("Angular velocity frame for " << name
                                                    << " is global frame.");
    else
      ROS_INFO_STREAM("Angular velocity frame for " << name
                                                    << " is local frame.");
  }

  /*
   * Check for the object calibration.
   */
  params = name + "/zero_pose/";

  /* Position calibration. */
  cal = cal && nh.getParam(params + "position/x", px);
  cal = cal && nh.getParam(params + "position/y", py);
  cal = cal && nh.getParam(params + "position/z", pz);

  if (cal)
  {
    ROS_INFO(
        "Position calibration for %s available:"
        " x = %f, y = %f, z = %f",
        name.c_str(), px, py, pz);

    zero_pose.setOrigin(tf::Vector3(px, py, pz));

    calibrated = true;
  }
  else
  {
    ROS_INFO(
        "Position calibration for %s unavailable,"
        " setting x = 0, y = 0, z = 0",
        name.c_str());

    zero_pose.setOrigin(tf::Vector3(0, 0, 0));
  }

  cal = true;

  /* Quaternion calibration. */
  cal = cal && nh.getParam(params + "rotation/w", qw);
  cal = cal && nh.getParam(params + "rotation/x", qx);
  cal = cal && nh.getParam(params + "rotation/y", qy);
  cal = cal && nh.getParam(params + "rotation/z", qz);

  has_quaternion = cal;

  if (!has_quaternion)
  {
    cal = true;

    /* If there was no quaternion, check for roll, pitch and yaw. */
    cal = cal && nh.getParam(params + "rotation/roll", rr);
    cal = cal && nh.getParam(params + "rotation/pitch", rp);
    cal = cal && nh.getParam(params + "rotation/yaw", ry);
  }

  /* Apply calibrations. */
  if (cal)
  {
    if (has_quaternion)
    {
      ROS_INFO(
          "Quaternion calibration for %s available:"
          " w = %f, x = %f, y = %f, z = %f",
          name.c_str(), qw, qx, qy, qz);

      zero_pose.setRotation(tf::Quaternion(qx, qy, qz, qw));
    }
    else
    {
      ROS_INFO(
          "RPY calibration for %s available:"
          " R = %f, P = %f, Y = %f",
          name.c_str(), rr, rp, ry);

      zero_pose.setRotation(tf::createQuaternionFromRPY(rr, rp, ry));
    }

    calibrated = calibrated & true;
  }
  else
  {
    ROS_INFO(
        "Rotation calibration for %s unavailable,"
        " setting quaternion w = 1, x = 0, y = 0, z = 0",
        name.c_str());

    zero_pose.setRotation(tf::Quaternion(0, 0, 0, 1));
  }

  /* Use the inverse to just have multiplications later. */
  zero_pose = zero_pose.inverse();

  /* Setup error checking. */
  ROS_INFO("Error publishing for %s", name.c_str());

  /* Get occluded setting. */
  cal = nh.getParam(name + "/errors/occluded", publish_occluded_errors);

  if (cal)
    ROS_INFO("\tOcclusions: Enabled");
  else
    ROS_INFO("\tOcclusions: Disabled");

  /* Get delta threshold setting. */
  bool dpos_available =
      nh.getParam(name + "/errors/max_delta_position", max_dpos);
  bool drot_available =
      nh.getParam(name + "/errors/max_delta_rotation", max_drot);

  if ((dpos_available || drot_available) && ((max_dpos > 0) || (max_drot > 0)))
  {
    ROS_INFO("\tFrame to frame thresholds: Enabled");

    if (dpos_available && (max_dpos > 0))
    {
      max_delta_position = max_dpos;
      ROS_INFO("\t\tPosition threshold: %f (m)", max_delta_position);
    }
    else
    {
      max_delta_position = 0;
      ROS_WARN("\t\tPosition threshold disabled.");
    }

    if (drot_available && (max_drot > 0))
    {
      max_delta_rotation = max_drot;
      ROS_INFO("\t\tRotation threshold: %f (rad)", max_delta_rotation);
    }
    else
    {
      max_delta_rotation = 0;
      ROS_WARN("\t\tRotation threshold disabled.");
    }
  }
  else
  {
    max_delta_position = 0;
    max_delta_rotation = 0;
    ROS_WARN("\tFrame to frame thresholds: Disabled");
  }
}

void ros_viconstream::ObjectPublisher::registerTF(const tf::Transform &tf)
{
  const auto current_time = ros::Time::now();

  if (first_frame)
  {
    last_tf      = tf;
    last_tf_time = current_time;
    first_frame  = false;
    delta_tf.setIdentity();
  }
  else
  {
    delta_tf          = last_tf.inverse() * tf;
    last_tf           = tf;
    last_tf_time      = current_time;
  }
}

geometry_msgs::Twist ros_viconstream::ObjectPublisher::getTwist()
{
  static unsigned old_cnt = 0;
  const auto dq = delta_tf.getRotation();

  const auto curr_speed = delta_tf.getOrigin() / frame_dt;
  auto curr_rate = 2 * tf::Vector3(dq.x(), dq.y(), dq.z()) / frame_dt;

  if (dq.w() < 0)
    curr_rate = -curr_rate;

  // Calculate 3-point median to reject jumps that comes from lags in
  // Ethernet connectivity
  auto med_filt = [](double a, double b, double c)
  {
    return std::max(std::min(a, b), std::min(std::max(a, b), c));
  };

  tf::Vector3 speed{
      med_filt(curr_speed.x(), old_speed[0].x(), old_speed[1].x()),
      med_filt(curr_speed.y(), old_speed[0].y(), old_speed[1].y()),
      med_filt(curr_speed.z(), old_speed[0].z(), old_speed[1].z())};

  tf::Vector3 rate{med_filt(curr_rate.x(), old_rate[0].x(), old_rate[1].x()),
                   med_filt(curr_rate.y(), old_rate[0].y(), old_rate[1].y()),
                   med_filt(curr_rate.z(), old_rate[0].z(), old_rate[1].z())};

  // Save old
  old_speed[old_cnt % 2] = curr_speed;
  old_rate[old_cnt % 2] = curr_rate;
  old_cnt++;


  const auto R = tf::Matrix3x3(last_tf.getRotation());

  /* Rotate into the correct frame. */
  if (global_frame_vel)
    speed        = R * speed;

  if (global_frame_rot)
    rate         = R * rate;

  geometry_msgs::Twist T;

  T.linear.x  = speed.x();
  T.linear.y  = speed.y();
  T.linear.z  = speed.z();
  T.angular.x = rate.x();
  T.angular.y = rate.y();
  T.angular.z = rate.z();

  return T;
}

bool ros_viconstream::ObjectPublisher::inThresholds()
{
  const double dpos = delta_tf.getOrigin().length();
  const double drot = std::abs(delta_tf.getRotation().getAngle());

  if ((max_delta_position > 0) && (dpos > max_delta_position))
  {
    ROS_ERROR_STREAM(
        "Outside delta position threshold! Delta position: " << dpos << " m");
    ROS_ERROR("Is the Vicon system calibrated and marker patterns unique?");

    return false;
  }
  else if ((max_delta_rotation > 0) && (drot > max_delta_rotation))
  {
    ROS_ERROR_STREAM(
        "Outside delta rotation threshold! Delta rotation: " << drot << " rad");
    ROS_ERROR("Is the Vicon system calibrated and marker patterns unique?");

    return false;
  }
  else
  {
    return true;
  }
}
