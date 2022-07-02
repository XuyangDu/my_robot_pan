#ifndef __PANTILT_CONF_H
#define __PANTILT_CONF_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class PantiltBroadcaster
{
public:
    PantiltBroadcaster(const ros::NodeHandle& handle, tf::TransformBroadcaster* tf);
    ~PantiltBroadcaster() {}

    void resetParams();
    void updateParams();
    // pan, tilt in degrees, elevator in meters
    void updateTransform(double panAng, double tiltAng, double elevHei, const ros::Time& time);

protected:
    ros::NodeHandle m_privNh;
    tf::TransformBroadcaster* m_tf;

    double base_h;
    double elevator_offx, elevator_offy, elevator_offz, elevator_lean;
    double supporter_offx, supporter_offy, supporter_offz;
    double supporter_roll, supporter_pitch, supporter_yaw;
    double armbase_offx, armbase_offy, armbase_offz;
    double kinect_offx, kinect_offy, kinect_offz;
    double kinect_roll, kinect_pitch, kinect_yaw;
    double cam1394_offx, cam1394_offy, cam1394_offz;
    double cam1394_roll, cam1394_pitch, cam1394_yaw;
    std::string kinect_frame, cam1394_frame;
};
typedef boost::shared_ptr<PantiltBroadcaster> PTBroaderPtr;


#endif
