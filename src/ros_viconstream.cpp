//          Copyright Emil Fresk 2015-2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "ros_viconstream/ros_viconstream.h"

/*********************************
 * Private members
 ********************************/

class ros_viconstream::ObjectPublisher
{
public:
  /* @brief Status of calibration from parameters. */
  bool calibrated;

  /* @brief Checks if the object is occluded for a longer time. */
  int occluded_counter;

  /* @brief Setting for publishing occluded errors. */
  bool publish_occluded_errors;

  /* @brief Max delta movement for error checking. */
  double max_delta_position;

  /* @brief Max delta rotation for error checking. */
  double max_delta_rotation;

  /* @brief Last pose used for error checking. */
  tf::Transform last_tf;

  /* @brief Checker for the first frame. */
  bool first_frame;

  /* @brief Holder of the zero pose of the object, used to remove offset from
   * an object to not have to fight with Vicon Tracker. */
  tf::Transform zero_pose;

  /* @brief Holder of the ROS publisher for this object's transforms. */
  ros::Publisher pub;

  /* @brief Holder of the ROS publisher for this object's errors. */
  ros::Publisher pub_error;

  /* @brief Name of the object on the form "subject name/segment name". */
  std::string name;

  /**
   * @brief   Creates a new object publisher object and loads a calibration
   *          of the zero pose if available.
   *
   * @param[in] nh            The node handle from ROS.
   * @param[in] objectPrefix  An prefix to the naming scheme (if wanted).
   * @param[in] subjectName   The subject name from the Vicon frame.
   * @param[in] segmentName   The segment name from the Vicon frame.
   */
  ObjectPublisher(ros::NodeHandle &nh, const std::string &objectPrefix,
                  const std::string &subjectName,
                  const std::string &segmentName)
      : calibrated(false), first_frame(true), zero_pose(tf::Pose::getIdentity())
  {
    std::string params;

    double px, py, pz, qw, qx, qy, qz, rr, rp, ry;
    double max_drot, max_dpos;
    bool cal = true;
    bool has_quaternion;
    occluded_counter = 0;

    /* Create object name. */
    if (objectPrefix.size() > 0)
      name = objectPrefix + "/" + subjectName + "/" + segmentName;
    else
      name = subjectName + "/" + segmentName;

    ROS_INFO("New object detected, adding %s", name.c_str());

    /* Advertise the data message. */
    pub = nh.advertise< geometry_msgs::TransformStamped >(name, 10);

    /* Advertise the error message. */
    pub_error = nh.advertise< std_msgs::String >(name + "/errors", 10);

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

    if ((dpos_available || drot_available) &&
        ((max_dpos > 0) || (max_drot > 0)))
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
        ROS_INFO("\t\tPosition threshold disabled.");
      }

      if (drot_available && (max_drot > 0))
      {
        max_delta_rotation = max_drot;
        ROS_INFO("\t\tRotation threshold: %f (rad)", max_delta_rotation);
      }
      else
      {
        max_delta_rotation = 0;
        ROS_INFO("\t\tRotation threshold disabled.");
      }
    }
    else
    {
      max_delta_position = 0;
      max_delta_rotation = 0;
      ROS_INFO("\tFrame to frame thresholds: Disabled");
    }
  }

  /**
   * @brief   Checks if the object is in the thresholds set by its parameters.
   *
   * @param[in] tf    Transform holding the position and rotation.
   *
   * @return True if inside the thresholds, else false.
   */
  bool inThresholds(const tf::Transform &tf)
  {
    if (first_frame)
    {
      last_tf     = tf;
      first_frame = false;
      return true;
    }
    else
    {
      const tf::Transform delta = last_tf.inverse() * tf;
      last_tf                   = tf;

      const double dpos = delta.getOrigin().length();
      const double drot = std::abs(delta.getRotation().getAngle());

      if ((max_delta_position > 0) && (dpos > max_delta_position))
      {
        ROS_ERROR_STREAM("Outside delta position threshold! Delta position: "
                         << dpos << " m");
        ROS_ERROR("Is the Vicon system calibrated and marker patterns unique?");

        return false;
      }
      else if ((max_delta_rotation > 0) && (drot > max_delta_rotation))
      {
        ROS_ERROR_STREAM("Outside delta rotation threshold! Delta rotation: "
                         << drot << " rad");
        ROS_ERROR("Is the Vicon system calibrated and marker patterns unique?");

        return false;
      }
      else
      {
        return true;
      }
    }
  }
};

ros_viconstream::ObjectPublisher &ros_viconstream::registerObject(
    const std::string &subjectName, const std::string &segmentName)
{
  std::string name(subjectName + "/" + segmentName);

  /* Search for the object in the map. */
  auto search_it = _objectList.find(name);

  if (search_it == _objectList.end())
  {
    /* Object is not available, create it. */
    ObjectPublisher op(_nh, _object_prefix, subjectName, segmentName);

    /* Add to the list of objects. */
    auto new_ob = _objectList.insert(
        std::pair< std::string, ObjectPublisher >(name, std::move(op)));

    /* Return the object reference. */
    return new_ob.first->second;
  }
  else
  {
    /* Return the object reference. */
    return search_it->second;
  }
}

void ros_viconstream::deadlineCallback()
{
  ROS_ERROR(
      "Vicon frames has timed out, is the Vicon system paused? If not, "
      "then the internal deadlock problem of the Vicon library has occurred. "
      "Restart the ros_viconstream node.");
}

void ros_viconstream::viconCallback(const Client &frame)
{
  tf::Transform tf;
  geometry_msgs::TransformStamped pub_tf;
  const ros::Time frame_curr_time = ros::Time::now();

  /* Reset and start the deadline again. */
  _dl.stop();
  _dl.start();

  /* Check the frame rate */
  if (_framerate == 0)
    _framerate = frame.GetFrameRate().FrameRateHz;

  /* Get the number of subjects. */
  const int subCount = frame.GetSubjectCount().SubjectCount;
  for (int subIndex = 0; subIndex < subCount; subIndex++)
  {
    /* Get the subject name. */
    const std::string subName = frame.GetSubjectName(subIndex).SubjectName;

    /* Get the number of segments in the subject. */
    const int segCount = frame.GetSegmentCount(subName).SegmentCount;
    for (int segIndex = 0; segIndex < segCount; segIndex++)
    {
      /* Get the segment name. */
      const std::string segName =
          frame.GetSegmentName(subName, segIndex).SegmentName;

      /* Extract the pose. */
      Output_GetSegmentGlobalTranslation translation =
          frame.GetSegmentGlobalTranslation(subName, segName);
      Output_GetSegmentGlobalRotationQuaternion rotation =
          frame.GetSegmentGlobalRotationQuaternion(subName, segName);

      /* Check if the object is registered, else add it. */
      ros_viconstream::ObjectPublisher &obj = registerObject(subName, segName);

      /* Check so the operation was successful. */
      if (translation.Result != Result::Success ||
          rotation.Result != Result::Success)
      {
        ROS_ERROR("Strange error, should not happen (result unsuccessful).");
        continue;
      }

      /* Check so the object is not occluded. */
      if (translation.Occluded || rotation.Occluded)
      {
        if (obj.publish_occluded_errors)
        {
          std_msgs::String err;
          err.data = "occluded";
          obj.pub_error.publish(err);
        }

        obj.occluded_counter++;

        if (obj.occluded_counter > (_framerate * 3))
        {
          obj.occluded_counter = 0;

          if (_framerate > 0)
            ROS_ERROR("Object '%s' has not been visible for 3 seconds.",
                      obj.name.c_str());
        }

        continue;
      }

      /* Move the Vicon measurement to a tf transform. */
      tf.setOrigin(tf::Vector3(translation.Translation[0] / 1000,    // x
                               translation.Translation[1] / 1000,    // y
                               translation.Translation[2] / 1000));  // z

      tf.setRotation(tf::Quaternion(rotation.Rotation[0],    // x
                                    rotation.Rotation[1],    // y
                                    rotation.Rotation[2],    // z
                                    rotation.Rotation[3]));  // w

      /* Apply calibration. */
      tf = obj.zero_pose * tf;

      /* Check so the object is in the thresholds. */
      if (!obj.inThresholds(tf))
      {
        std_msgs::String err;
        err.data = "thresholds";
        obj.pub_error.publish(err);

        continue;
      }

      /* Save all transform for the TransformBroadcaster. */
      _tf_list.push_back(tf::StampedTransform(tf, frame_curr_time,
                                              _id_reference_frame, obj.name));

      /* Publish the geometry_msg transform. */
      tf::transformStampedTFToMsg(_tf_list.back(), pub_tf);
      obj.pub.publish(pub_tf);
    }
  }

  /* Publish the tf list. */
  _tf_broadcaster.sendTransform(_tf_list);

  /* Clear for next frame. */
  _tf_list.clear();
}

/*********************************
 * Public members
 ********************************/

ros_viconstream::ros_viconstream(std::ostream &os)
    : _framerate(0), _nh("~"), _dl([&]() { this->deadlineCallback(); }, 500)
{
  /* Get the Vicon URL. */
  std::string vicon_url;
  std::string vicon_mode;

  if (!_nh.getParam("vicon_url", vicon_url))
  {
    ROS_WARN("No Vicon URL found defaulting to localhost.");
    vicon_url = "localhost:801";
  }

  if (!_nh.getParam("vicon_streammode", vicon_mode))
  {
    ROS_WARN("No Vicon StreamMode found defaulting to ServerPush.");
    vicon_mode = "ServerPush";
  }

  /* Check for naming prefix. */
  if (!_nh.getParam("object_prefix", _object_prefix))
    _object_prefix.clear();

  /* Check for reference frame naming. */
  if (!_nh.getParam("id_reference_frame", _id_reference_frame))
  {
    ROS_INFO("No reference frame specified, defaulting to \"/world\".");
    _id_reference_frame = "/world";
  }

  /* Connect to Vicon. */
  _vs = std::unique_ptr< libviconstream::arbiter >{
      new libviconstream::arbiter(vicon_url, os)};

  /* Subscribe to the vicon frames. */
  _vs->registerCallback(
      [&](const Client &frame) { this->viconCallback(frame); });

  auto mode = StreamMode::ServerPush;

  if (vicon_mode == "ClientPull")
    mode = StreamMode::ClientPull;
  else if (vicon_mode == "ServerPush")
    mode = StreamMode::ServerPush;
  else
    ROS_ERROR("Invalid mode detected, was: '%s'. Defaulting to ServerPush.",
              vicon_mode.c_str());

  /* Allocate space in the TF list */
  _tf_list.reserve(100);

  /* Enable the stream and check for errors. */
  if (!_vs->enableStream(true, false, false, false, mode))
  {
    ROS_ERROR("Unable to connect to vicon!");

    return;
  }
}

ros_viconstream::~ros_viconstream()
{
  if (_vs != NULL)
  {
    _vs->disableStream();
  }
}
