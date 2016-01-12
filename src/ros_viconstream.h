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

    class ObjectPublisher
    {
    public:
        bool calibrated;
        tf::Transform zero_pose;
        ros::Publisher pub;

        ObjectPublisher(ros::NodeHandle &nh,
                        std::string subjectName,
                        std::string segmentName)
            : calibrated(false), zero_pose(tf::Pose::getIdentity())
        {
            std::string name = subjectName + "/" + segmentName;
            pub = nh.advertise<geometry_msgs::TransformStamped> (name, 5);
        }
    };

    /* @brief Map holding the Vicon objects and their respective publisher. */
    std::map<std::string, ObjectPublisher> _objectList;

    /* @brief The local ROS node handle. */
    ros::NodeHandle _nh;

    /* @brief The TF broadcaste. */
    tf::TransformBroadcaster _tf_broadcaster;

    ViconStream::ViconStream *_vs;
    int _cb_id;
    void viconCallback(const Client &frame);

public:
    ROS_ViconStream(std::ostream &os);
    ~ROS_ViconStream();

};
