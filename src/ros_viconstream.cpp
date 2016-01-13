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
        std::string params;

        double px, py, pz, qw, qx, qy, qz, rr, rp, ry;
        bool cal = true;
        bool has_quaternion;

        /* Create object name. */
        if (objectPrefix.size() > 0)
            name = objectPrefix + "/" + subjectName + "/" + segmentName;
        else
            name = subjectName + "/" + segmentName;

        /* Advertise the message. */
        pub = nh.advertise<geometry_msgs::TransformStamped> (name, 5);

        /*
         * Check for the object calibration.
         */
        params = name + "/zero_pose/";

        /* Position calibration. */
        cal = cal && nh.getParam(params + "position/x", px);
        cal = cal && nh.getParam(params + "position/y", py);
        cal = cal && nh.getParam(params + "position/z", pz);

        if (cal)
        {
            ROS_INFO("Position calibration for %s available:"
                     " x = %f, y = %f, z = %f",
                     name.c_str(), px, py, pz);

            zero_pose.setOrigin( tf::Vector3(px, py, pz) );
        }
        else
        {
            ROS_WARN("Position calibration for %s unavailable,"
                     " setting x = 0, y = 0, z = 0",
                     name.c_str());

            zero_pose.setOrigin( tf::Vector3(0, 0, 0) );
        }

        cal = true;

        /* Quaternion calibration. */
        cal = cal && nh.getParam(params + "rotation/w", qw);
        cal = cal && nh.getParam(params + "rotation/x", qx);
        cal = cal && nh.getParam(params + "rotation/y", qy);
        cal = cal && nh.getParam(params + "rotation/z", qz);

        has_quaternion = cal;

        if (!has_quaternion)
        {
            cal = true;

            /* If there was no quaternion, check for roll, pitch and yaw. */
            cal = cal && nh.getParam(params + "rotation/roll", rr);
            cal = cal && nh.getParam(params + "rotation/pitch", rp);
            cal = cal && nh.getParam(params + "rotation/yaw", ry);
        }

        /* Apply calibrations. */
        if (cal)
        {
            if (has_quaternion)
            {
                ROS_INFO("Quaternion calibration for %s available:"
                         " w = %f, x = %f, y = %f, z = %f",
                         name.c_str(), qw, qx, qy, qz);

                zero_pose.setRotation( tf::Quaternion(qx, qy, qz, qw) );
            }
            else
            {
                ROS_INFO("RPY calibration for %s available:"
                         " R = %f, P = %f, Y = %f",
                         name.c_str(), rr, rp, ry);

                zero_pose.setRotation( tf::createQuaternionFromRPY(rr, rp, ry) );
            }
        }
        else
        {
            ROS_WARN("Rotation calibration for %s unavailable,"
                     " setting quaternion w = 1, x = 0, y = 0, z = 0",
                     name.c_str());

            zero_pose.setRotation( tf::Quaternion(0, 0, 0, 1) );
        }

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
