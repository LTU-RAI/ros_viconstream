//          Copyright Emil Fresk 2015-2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "ros_viconstream/ros_viconstream.h"

ros_viconstream::ObjectPublisher &ros_viconstream::registerObject(
    const std::string &subjectName, const std::string &segmentName,
    const double framerate_hz)
{
  const std::string name(subjectName + "/" + segmentName);

  /* Search for the object in the map. */
  auto search_it = _objectList.find(name);

  if (search_it == _objectList.end())
  {
    /* Object is not available, create it. */
    ObjectPublisher op(_nh, _object_prefix, subjectName, segmentName,
                       framerate_hz);

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
  nav_msgs::Odometry pub_odom;
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
      ros_viconstream::ObjectPublisher &obj =
          registerObject(subName, segName, _framerate);

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

      /* Register the TF to the object. */
      obj.registerTF(tf);

      /* Check so the object is in the thresholds. */
      if (!obj.inThresholds())
      {
        std_msgs::String err;
        err.data = "thresholds";
        obj.pub_error.publish(err);

        continue;
      }

      /* Save all transform for the TransformBroadcaster. */
      _tf_list.push_back(tf::StampedTransform(tf, frame_curr_time,
                                              _id_reference_frame, obj.name));

      /* Publish the geometry_msg::TransformStapmed */
      tf::transformStampedTFToMsg(_tf_list.back(), pub_tf);
      obj.pub.publish(pub_tf);

      /* Publish the nav_msgs::Odometry */
      pub_odom.header = pub_tf.header;
      pub_odom.child_frame_id = pub_tf.child_frame_id;

      pub_odom.pose.pose.position.x = pub_tf.transform.translation.x;
      pub_odom.pose.pose.position.y = pub_tf.transform.translation.y;
      pub_odom.pose.pose.position.z = pub_tf.transform.translation.z;

      pub_odom.pose.pose.orientation = pub_tf.transform.rotation;
      pub_odom.twist.twist = obj.getTwist();

      obj.pub_odom.publish(pub_odom);
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
  ROS_INFO("Connecting to the Vicion system...");
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
