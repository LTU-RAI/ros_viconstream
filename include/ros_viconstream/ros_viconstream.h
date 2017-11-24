//          Copyright Emil Fresk 2015-2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef _ROS_VICONSTREAM_H
#define _ROS_VICONSTREAM_H

#include <iostream>
#include <map>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include "libviconstream/viconstream.h"
#include "deadline.h"

class ros_viconstream
{
private:
  /* @brief Class defining an object publisher. */
  class ObjectPublisher
  {
  public:
    /* @brief Status of calibration from parameters. */
    bool calibrated;

    /* @brief Checks if the object is occluded for a longer time. */
    int occluded_counter;

    /* @brief Setting for publishing occluded errors. */
    bool publish_occluded_errors;

    /* @brief Setting for if the velocity is in the global frame. */
    bool global_frame_vel;

    /* @brief Setting for if the angular rate is in the global frame. */
    bool global_frame_rot;

    /* @brief Max delta movement for error checking. */
    double max_delta_position;

    /* @brief Max delta rotation for error checking. */
    double max_delta_rotation;

    /* @brief Last pose used odometry and for error checking. */
    tf::Transform last_tf;

    /* @brief Delta frame used odometry. */
    tf::Transform delta_tf;

    /* @brief Time for the last frame. */
    ros::Time last_tf_time;

    /* @brief Duration for the delta frame. */
    ros::Duration duration_delta_tf;

    /* @brief Checker for the first frame. */
    bool first_frame;

    /* @brief Holder of the zero pose of the object, used to remove offset from
     * an object to not have to fight with Vicon Tracker. */
    tf::Transform zero_pose;

    /* @brief Holder of the ROS publisher for this object's transforms. */
    ros::Publisher pub;

    /* @brief Holder of the ROS publisher for this object's odometry. */
    ros::Publisher pub_odom;

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
                    const std::string &segmentName);

    /**
     * @brief   Registers the current Transform of the object.
     *
     * @param[in] tf    Transform holding the position and rotation.
     */
    void registerTF(const tf::Transform &tf);


    /**
     * @brief   Calculates the twist from the last frame.
     *
     * @return  The calculated twist.
     */
    geometry_msgs::Twist getTwist();

    /**
     * @brief   Checks if the object is in the thresholds set by its parameters.
     *
     * @return True if inside the thresholds, else false.
     */
    bool inThresholds();
  };

  /* @brief ID of the reference frame (Vicon area). */
  std::string _id_reference_frame;

  /* @brief Frame rate for dropout counter. */
  unsigned int _framerate;

  /* @brief Object name prefix. */
  std::string _object_prefix;

  /* @brief Map holding the Vicon objects and their respective publisher. */
  std::map< std::string, ObjectPublisher > _objectList;

  /* @brief The local ROS node handle. */
  ros::NodeHandle _nh;

  /* @brief The TF broadcaster. */
  tf::TransformBroadcaster _tf_broadcaster;

  /* @brief ViconStream object. */
  std::unique_ptr< libviconstream::arbiter > _vs;

  /* @brief Deadline checking Vicon's library for deadlocks. */
  Deadline::Deadline _dl;

  /* @brief list of TFs for publishing, to not reallocate all the time. */
  std::vector< tf::StampedTransform > _tf_list;

  /**
   * @brief   Registers or finds the corresponding Vicon object.
   *
   * @param[in] subjectName   The subject name from the Vicon frame.
   * @param[in] segmentName   The segment name from the Vicon frame.
   */
  ros_viconstream::ObjectPublisher &registerObject(
      const std::string &subjectName, const std::string &segmentName);
  /**
   * @brief   Callback for the deadline supervisor.
   */
  void deadlineCallback();

  /**
   * @brief   Callback for the viconstream.
   *
   * @param[in] frame Read only frame from the viconstream.
   */
  void viconCallback(const Client &frame);

public:
  /**
   * @brief   Constructor for the ros_viconstream.
   *
   * @param[in] os Reference to the log output stream.
   */
  ros_viconstream(std::ostream &os);

  /**
   * @brief   Destructor that handles the graceful exit of the Vicon.
   */
  ~ros_viconstream();
};

#endif
