/****************************************************************************
*
* Copyright (C) 2016 Emil Fresk.
* All rights reserved.
*
* This file is part of the ROS ViconStream node.
*
* GNU Lesser General Public License Usage
* This file may be used under the terms of the GNU Lesser
* General Public License version 3.0 as published by the Free Software
* Foundation and appearing in the file LICENSE included in the
* packaging of this file.  Please review the following information to
* ensure the GNU Lesser General Public License version 3.0 requirements
* will be met: http://www.gnu.org/licenses/lgpl-3.0.html.
*
* If you have questions regarding the use of this file, please contact
* Emil Fresk at emil.fresk@gmail.com.
*
****************************************************************************/

#ifndef _ROS_VICONSTREAM_H
#define _ROS_VICONSTREAM_H

#include <iostream>
#include <map>
#include <vector>
#include <thread>
#include <atomic>

#include "ros/ros.h"
#include "viconstream/viconstream.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"

#include "deadline.h"

class ROS_ViconStream
{
private:

    /* @brief ID of the reference frame (Vicon area). */
    std::string _id_reference_frame;

    /* @brief Object name prefix. */
    std::string _object_prefix;

    /* @brief Class defining an object publisher. */
    class ObjectPublisher;

    /* @brief Map holding the Vicon objects and their respective publisher. */
    std::map<std::string, ObjectPublisher> _objectList;

    /* @brief The local ROS node handle. */
    ros::NodeHandle _nh;

    /* @brief The TF broadcaster. */
    tf::TransformBroadcaster _tf_broadcaster;

    /* @brief ViconStream object. */
    ViconStream::ViconStream *_vs;

    /* @biref Deadline checking Vicon's library for deadlocks. */
    Deadline::Deadline _dl;

    /* Deadline ID. */
    std::atomic<unsigned int> _dl_id;

    /**
     * @brief   Registers or finds the corresponding Vicon object.
     *
     * @param[in] subjectName   The subject name from the Vicon frame.
     * @param[in] segmentName   The segment name from the Vicon frame.
     */
    ROS_ViconStream::ObjectPublisher& registerObject(
            const std::string &subjectName,
            const std::string &segmentName);
    /**
     * @brief   Callback for the deadline supervisor.
     */
    void deadlineCallback();

    /**
     * @brief   Callback for the ViconStream.
     *
     * @param[in] frame Read only frame from the ViconStream.
     */
    void viconCallback(const Client &frame);

public:

    /**
     * @brief   Constructor for the ROS_ViconStream.
     *
     * @param[in] os Reference to the log output stream.
     */
    ROS_ViconStream(std::ostream &os);

    /**
     * @brief   Destructor that handles the graceful exit of the Vicon.
     */
    ~ROS_ViconStream();

};

#endif
