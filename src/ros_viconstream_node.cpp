#include <iostream>

#include "ros_viconstream.h"

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

    /* Start the ViconStream. */
    ROS_ViconStream vs(cout);

    /* Let ROS run. */
    ros::spin();

    return 0;
}
