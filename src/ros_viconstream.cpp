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
    std::string name;

    ObjectPublisher(ros::NodeHandle &nh,
                    std::string &objectPrefix,
                    std::string &subjectName,
                    std::string &segmentName)
        : calibrated(false), zero_pose(tf::Pose::getIdentity())
    {
        std::string params;

        double px, py, pz, qw, qx, qy, qz, rr, rp, ry;
        bool cal = true;
        bool has_quaternion;

        /* Create object name. */
        if (objectPrefix.size() > 0)
            name = objectPrefix + "/" + subjectName + "/" + segmentName;
        else
            name = subjectName + "/" + segmentName;

        ROS_INFO("New object detected, adding %s", name.c_str());

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

        /* Use the inverse to just have multiplications later. */
        zero_pose = zero_pose.inverse();

    }
};

ROS_ViconStream::ObjectPublisher& ROS_ViconStream::registerObject(
    std::string &subjectName,
    std::string &segmentName)
{
    std::string name(subjectName + "/" + segmentName);

    /* Search for the object in the map. */
    auto search_it = _objectList.find(name);

    if (search_it == _objectList.end())
    {
        /* Object is not available, create it. */
        ObjectPublisher op(_nh, _object_prefix, subjectName, segmentName);

        /* Add to the list of objects. */
        auto new_ob = _objectList.insert(
            std::pair<std::string, ObjectPublisher> (name, std::move(op))
        );

        /* Return the object reference. */
        return new_ob.first->second;
    }
    else
    {
        /* Return the object reference. */
        return search_it->second;
    }
}

void ROS_ViconStream::viconCallback(const Client &frame)
{
    tf::Transform tf;
    std::vector<tf::StampedTransform> tf_list;
    geometry_msgs::TransformStamped pub_tf;
    const ros::Time frame_curr_time = ros::Time::now();

    /* Get the number of subjects. */
    const int subCount = frame.GetSubjectCount().SubjectCount;
    for( int subIndex = 0; subIndex < subCount; subIndex++ )
    {
        /* Get the subject name. */
        std::string subName = frame.GetSubjectName( subIndex ).SubjectName;

        /* Get the number of segments in the subject. */
        const int segCount = frame.GetSegmentCount( subName ).SegmentCount;
        for( int segIndex = 0; segIndex < segCount; segIndex++ )
        {
            /* Get the segment name. */
            std::string segName = frame.GetSegmentName( subName,
                                                        segIndex ).SegmentName;

            /* Extract the pose. */
            Output_GetSegmentGlobalTranslation translation =
                frame.GetSegmentGlobalTranslation(subName, segName);
            Output_GetSegmentGlobalRotationQuaternion rotation =
                frame.GetSegmentGlobalRotationQuaternion(subName, segName);

            /* Check if the object is registered, else add it. */
            ROS_ViconStream::ObjectPublisher &obj = registerObject(subName,
                                                                   segName);

            /* Check so the operation was successful. */
            if (translation.Result != Result::Success ||
                rotation.Result != Result::Success)
                continue;

            /* Check so the object is not occluded. */
            if (translation.Occluded || rotation.Occluded)
                continue;

            /* Move the Vicon measurement to a tf transform. */
            tf.setOrigin( tf::Vector3(translation.Translation[0] / 1000,
                                      translation.Translation[1] / 1000,
                                      translation.Translation[2] / 1000) );

            tf.setRotation( tf::Quaternion(rotation.Rotation[0],
                                           rotation.Rotation[1],
                                           rotation.Rotation[2],
                                           rotation.Rotation[3]) );

            /* Apply calibration. */
            tf *= obj.zero_pose;

            /* Save all transform for the TransformBroadcaster. */
            tf_list.push_back( tf::StampedTransform( tf,
                                                     frame_curr_time,
                                                     _id_reference_frame,
                                                     obj.name) );

            /* Publish the geometry_msg transform. */
            tf::transformStampedTFToMsg( tf_list.back(), pub_tf );
            obj.pub.publish( pub_tf );
        }
    }
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
