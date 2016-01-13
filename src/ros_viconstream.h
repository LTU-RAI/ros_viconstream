#include <iostream>
#include <map>
#include <vector>
#include <thread>

#include "ros/ros.h"
#include "viconstream/viconstream.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"




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

    ROS_ViconStream::ObjectPublisher&registerObject(std::string &subjectName,
                                                    std::string &segmentName);
    void viconCallback(const Client &frame);

public:
    ROS_ViconStream(std::ostream &os);
    ~ROS_ViconStream();

};
