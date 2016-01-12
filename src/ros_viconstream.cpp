#include "ros_viconstream.h"


/*********************************
 * Private members
 ********************************/

class ROS_ViconStream::ObjectPublisher
{
public:
    bool calibrated;
    tf::Transform zero_pose;
    ros::Publisher pub;

    ObjectPublisher(ros::NodeHandle &nh,
                    std::string &objectPrefix,
                    std::string &subjectName,
                    std::string &segmentName)
        : calibrated(false), zero_pose(tf::Pose::getIdentity())
    {
        std::string name;

        /* Create object name. */
        if (objectPrefix.size() > 0)
            name = objectPrefix + "/" + subjectName + "/" + segmentName;
        else
            name = subjectName + "/" + segmentName;

        /* Advertise the message. */
        pub = nh.advertise<geometry_msgs::TransformStamped> (name, 5);

        /* Check for the object calibration. */
    }
};

void ROS_ViconStream::viconCallback(const Client &frame)
{
    (void) frame;
}

/*********************************
 * Public members
 ********************************/

ROS_ViconStream::ROS_ViconStream(std::ostream &os) : _nh("~"), _vs(NULL)
{
    /* Get the Vicon URL. */
    std::string vicon_url;

    if (!_nh.getParam("vicon_url", vicon_url))
    {
        ROS_WARN("No Vicon URL found defaulting to localhost.");
        vicon_url = "localhost:801";
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
    _vs->registerCallback([&] (const Client &frame) {

        /* Using lambda to get around the static. */
        this->viconCallback(frame);
    });

    /* Enable the stream and check for errors. */
    if (!_vs->enableStream(true, false, false, false, StreamMode::ServerPush))
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
