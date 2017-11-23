//          Copyright Emil Fresk 2015-2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "ros_viconstream/ros_viconstream.h"

/*********************************
 * Private members
 ********************************/

class ROS_ViconStream::ObjectPublisher
{
public:
  /* @brief Status of calibration from parameters. */
  bool calibrated;

  /* @brief Checks if the object is occluded for a longer time. */
  int occluded_counter;

  /* @brief Setting for publishing occluded errors. */
  bool publish_occluded_errors;

  /* @brief Setting for publishing bounding box errors. */
  bool publish_bounding_box_errors;

  /* @brief Bounding box for error checking. */
  struct
  {
    struct
    {
      double x, y, z;
    } min, max;
  } bounding_box;

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
      : calibrated(false), zero_pose(tf::Pose::getIdentity())
  {
    std::string params;

    double px, py, pz, qw, qx, qy, qz, rr, rp, ry;
    double c1x, c1y, c1z, c2x, c2y, c2z;
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
      ROS_WARN(
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
      ROS_WARN(
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

    /* Get bounding box setting. */
    cal = true;

    cal = cal && nh.getParam(name + "/errors/boundingbox/min/x", c1x);
    cal = cal && nh.getParam(name + "/errors/boundingbox/min/y", c1y);
    cal = cal && nh.getParam(name + "/errors/boundingbox/min/z", c1z);

    cal = cal && nh.getParam(name + "/errors/boundingbox/max/x", c2x);
    cal = cal && nh.getParam(name + "/errors/boundingbox/max/y", c2y);
    cal = cal && nh.getParam(name + "/errors/boundingbox/max/z", c2z);

    publish_bounding_box_errors = cal;

    if (cal && (c1x >= c2x))
    {
      ROS_WARN(
          "\tBounding box: X coordinate, min is bigger than, or equal to max.");
      cal = false;
    }
    if (cal && (c1y >= c2y))
    {
      ROS_WARN(
          "\tBounding box: Y coordinate, min is bigger than, or equal to max.");
      cal = false;
    }
    if (cal && (c1z >= c2z))
    {
      ROS_WARN(
          "\tBounding box: Z coordinate, min is bigger than, or equal to max.");
      cal = false;
    }

    if (cal)
    {
      bounding_box.min.x = c1x;
      bounding_box.min.y = c1y;
      bounding_box.min.z = c1z;

      bounding_box.max.x = c2x;
      bounding_box.max.y = c2y;
      bounding_box.max.z = c2z;

      ROS_INFO("\tBounding box: Enabled");
      ROS_INFO("\t\tMin: %f (x), %f (y), %f (z)", c1x, c1y, c1z);
      ROS_INFO("\t\tMax: %f (x), %f (y), %f (z)", c2x, c2y, c2z);
    }
    else
    {
      ROS_INFO("\tBounding box: Disabled");
    }
  }

  /**
   * @brief   Checks if the object is in the bounding box set by its
   *          parameters.
   *
   * @param[in] x     X position.
   * @param[in] y     Y position.
   * @param[in] z     Z position.
   *
   * @return True if inside the box, else false.
   */
  bool inBoundingBox(const double x, const double y, const double z)
  {
    if ((x >= bounding_box.min.x) && (x <= bounding_box.max.x) &&
        (y >= bounding_box.min.y) && (y <= bounding_box.max.y) &&
        (z >= bounding_box.min.z) && (z <= bounding_box.max.z))
      return true;
    else
      return false;
  }

  /**
   * @brief   Checks if the object is in the bounding box set by its
   *          parameters.
   *
   * @param[in] tf    Transform holding the position.
   *
   * @return True if inside the box, else false.
   */
  bool inBoundingBox(const tf::Transform &tf)
  {
    geometry_msgs::Transform pose;
    tf::transformTFToMsg(tf, pose);

    return inBoundingBox(pose.translation.x, pose.translation.y,
                         pose.translation.z);
  }
};

ROS_ViconStream::ObjectPublisher &ROS_ViconStream::registerObject(
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

void ROS_ViconStream::deadlineCallback()
{
  ROS_ERROR("The viconCallback has timed out, is the Vicon system paused?");
}

void ROS_ViconStream::viconCallback(const Client &frame)
{
  tf::Transform tf;
  std::vector< tf::StampedTransform > tf_list;
  geometry_msgs::TransformStamped pub_tf;
  const ros::Time frame_curr_time = ros::Time::now();

  /* Allocate the number of TFs plus some extra. */
  tf_list.reserve(_objectList.size() + 10);

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
      ROS_ViconStream::ObjectPublisher &obj = registerObject(subName, segName);

      /* Check so the operation was successful. */
      if (translation.Result != Result::Success ||
          rotation.Result != Result::Success)
      {
        ROS_WARN("Strange error, should not happen (result unsucessful).");
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
            ROS_WARN("Object '%s' has not been visible for 3 seconds.",
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

      /* Check so the object is in the bounding box. */
      if (obj.publish_bounding_box_errors)
      {
        if (!obj.inBoundingBox(tf))
        {
          std_msgs::String err;
          err.data = "bounding_box";
          obj.pub_error.publish(err);

          continue;
        }
      }

      /* Save all transform for the TransformBroadcaster. */
      tf_list.push_back(tf::StampedTransform(tf, frame_curr_time,
                                             _id_reference_frame, obj.name));

      /* Publish the geometry_msg transform. */
      tf::transformStampedTFToMsg(tf_list.back(), pub_tf);
      obj.pub.publish(pub_tf);
    }
  }

  /* Publish the tf list. */
  _tf_broadcaster.sendTransform(tf_list);
}

/*********************************
 * Public members
 ********************************/

ROS_ViconStream::ROS_ViconStream(std::ostream &os)
    : _framerate(0)
    , _nh("~")
    , _vs(NULL)
    , _dl([&]() { this->deadlineCallback(); }, 500)
    , _dl_id(0)
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
    _id_reference_frame = "/world";
  }

  /* Connect to Vicon. */
  _vs = new ViconStream::ViconStream(vicon_url, os);

  /* Subscribe to the vicon frames. */
  _vs->registerCallback([&](const Client &frame) {

    /* Using lambda to get around the static. */
    this->viconCallback(frame);
  });

  auto mode = StreamMode::ServerPush;

  if (vicon_mode == "ClientPull")
    mode = StreamMode::ClientPull;
  else if (vicon_mode == "ServerPush")
    mode = StreamMode::ServerPush;
  else
    ROS_WARN("Invalid mode detected, was: '%s'. Defaulting to ServerPush.",
             vicon_mode.c_str());

  /* Enable the stream and check for errors. */
  if (!_vs->enableStream(true, false, false, false, mode))
  {
    ROS_ERROR("Unable to connect to vicon!");

    return;
  }
}

ROS_ViconStream::~ROS_ViconStream()
{
  if (_vs != NULL)
  {
    _vs->disableStream();
    delete _vs;
  }
}
