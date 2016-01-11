#include <iostream>

#include "ros/ros.h"
#include "viconstream/viconstream.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;

void vicon_cb(const Client &frame)
{
    cout << "Frame: " << frame.GetFrameNumber().FrameNumber << endl;
}

int main(int argc, char *argv[])
{
    /*
     * Initializing ROS
     */
    ROS_INFO("Initializing Vicon...");
    ros::init(argc, argv, "ros_viconstream");
    ros::NodeHandle n("~");

    /* Get the Vicon URL. */
    std::string vicon_url;

    if (!n.getParam("vicon_url", vicon_url))
    {
        ROS_WARN("No Vicon URL found defaulting to localhost.");
        vicon_url = "localhost:801";
    }

    /* Connect to vicon. */
    ViconStream::ViconStream vs(vicon_url, cout);

    /* Subscribe to the vicon frames. */
    vs.registerCallback(vicon_cb);

    /* Enable the stream and check for errors. */
    if (!vs.enableStream(true, false, false, false, StreamMode::ServerPush))
    {
        ROS_ERROR("Unable to connect to vicon!");

        return 0;
    }

    ros::spin();

    /* Disable the stream before closing. */
    vs.disableStream();

    return 0;
}
