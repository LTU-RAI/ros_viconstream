#include "ros_viconstream.h"


void ROS_ViconStream::viconCallback(const Client &frame)
{
    (void) frame;
}

ROS_ViconStream::ROS_ViconStream(std::ostream &os) : _nh("~"), _vs(NULL)
{
    /* Get the Vicon URL. */
    std::string vicon_url;

    if (!_nh.getParam("vicon_url", vicon_url))
    {
        ROS_WARN("No Vicon URL found defaulting to localhost.");
        vicon_url = "localhost:801";
    }

    /* Connect to Vicon. */
    _vs = new ViconStream::ViconStream(vicon_url, os);

    /* Subscribe to the vicon frames. */
    _cb_id = _vs->registerCallback([&] (const Client &frame) {

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
