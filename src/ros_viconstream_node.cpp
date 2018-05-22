//          Copyright Emil Fresk 2015-2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <iostream>
#include "ros_viconstream/ros_viconstream.h"

using namespace std;

#ifdef NDEBUG
class NullBuffer : public std::streambuf
{
public:
  int overflow(int c)
  {
    return c;
  }
};
#endif

int main(int argc, char *argv[])
{
  /*
   * Initializing ROS
   */
  ROS_INFO("Initializing Vicon...");
  ros::init(argc, argv, "ros_viconstream");

#ifdef NDEBUG
  NullBuffer null_buffer;
  std::ostream null_stream(&null_buffer);

  /* Start the ViconStream. */
  ros_viconstream vs(null_stream);
#else
  ros_viconstream vs(cout);
#endif

  /* Let ROS run. */
  ros::spin();

  return 0;
}
