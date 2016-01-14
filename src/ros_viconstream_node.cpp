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

#include <iostream>
#include "ros_viconstream.h"

using namespace std;

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
