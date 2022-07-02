#include <string>
#include <XmlRpc.h>

#include "xform.hpp"

#include "pantiltBroadcaster.h"

using namespace std;
using namespace xform;

static const std::string g_baseFrame = "/base_link";
static const std::string g_kinectFrame = "/openni_camera";
static const std::string g_cam1394Frame = "/camera_1394";

PantiltBroadcaster::PantiltBroadcaster(const ros::NodeHandle &handle, tf::TransformBroadcaster *tf):
    m_privNh(handle),
    m_tf(tf)
{
    resetParams();
}

void PantiltBroadcaster::resetParams()
{
    base_h = 0.15;
    elevator_offx = -0.025;
    elevator_offy = 0.005;
    elevator_offz = 0.11;
    elevator_lean = 0.;
    supporter_offx = 0.15;
    supporter_offy = 0.;
    supporter_offz = 0.395;
    supporter_roll = 0.;
    supporter_pitch = 0.;
    supporter_yaw = 0.;
    armbase_offx = 0.21;
    armbase_offy = 0.;
    armbase_offz = 0.025;
    kinect_offx = 0.04;
    kinect_offy = -0.012;
    kinect_offz = 0.19;
    kinect_roll = 0.0;
    kinect_pitch = 0.0;
    kinect_yaw = 0.0;
    cam1394_offx = 0.04;
    cam1394_offy = -0.019;
    cam1394_offz = 0.128;
    cam1394_roll = 0.0;
    cam1394_pitch = 0.0;
    cam1394_yaw = 0.0;
    kinect_frame = g_kinectFrame;
    cam1394_frame = g_cam1394Frame;
}

void PantiltBroadcaster::updateParams()
{
    // update param
    string key;
    if (m_privNh.ok() && m_privNh.searchParam("PanTilt", key))
    {
        if (!m_privNh.getParamCached(key + "/baseHeight", base_h))
            ROS_WARN("Param not get[%s]", (key + "/baseHeight").c_str());

        m_privNh.getParamCached(key + "/elevator_offx", elevator_offx);
        m_privNh.getParamCached(key + "/elevator_offy", elevator_offy);
        m_privNh.getParamCached(key + "/elevator_offz", elevator_offz);
        m_privNh.getParamCached(key + "/elevator_lean", elevator_lean);

        m_privNh.getParamCached(key + "/supporter_offx", supporter_offx);
        m_privNh.getParamCached(key + "/supporter_offy", supporter_offy);
        m_privNh.getParamCached(key + "/supporter_offz", supporter_offz);

        m_privNh.getParamCached(key + "/supporter_roll", supporter_roll);
        m_privNh.getParamCached(key + "/supporter_pitch", supporter_pitch);
        m_privNh.getParamCached(key + "/supporter_yaw", supporter_yaw);

        m_privNh.getParamCached(key + "/armbase_offx", armbase_offx);
        m_privNh.getParamCached(key + "/armbase_offy", armbase_offy);
        m_privNh.getParamCached(key + "/armbase_offz", armbase_offz);

        m_privNh.getParamCached(key + "/kinect_offx", kinect_offx);
        m_privNh.getParamCached(key + "/kinect_offy", kinect_offy);
        m_privNh.getParamCached(key + "/kinect_offz", kinect_offz);

        m_privNh.getParamCached(key + "/kinect_roll", kinect_roll);
        m_privNh.getParamCached(key + "/kinect_pitch", kinect_pitch);
        m_privNh.getParamCached(key + "/kinect_yaw", kinect_yaw);

        m_privNh.getParamCached(key + "/cam1394_offx", cam1394_offx);
        m_privNh.getParamCached(key + "/cam1394_offy", cam1394_offy);
        m_privNh.getParamCached(key + "/cam1394_offz", cam1394_offz);

        m_privNh.getParamCached(key + "/cam1394_roll", cam1394_roll);
        m_privNh.getParamCached(key + "/cam1394_pitch", cam1394_pitch);
        m_privNh.getParamCached(key + "/cam1394_yaw", cam1394_yaw);

        if (!m_privNh.getParamCached(key + "/kinect_frame", kinect_frame))
            kinect_frame = g_kinectFrame;
        if (!m_privNh.getParamCached(key + "/camera1394_frame", cam1394_frame))
            cam1394_frame = g_cam1394Frame;
    }
    else
        ROS_ERROR("Param[PanTilt] not found!");
}

void PantiltBroadcaster::updateTransform(double panAng, double tiltAng, double elevHei, const ros::Time &time)
{
    Vector3D elevator(elevator_offx, elevator_offy, elevator_offz);
    Vector3D supporter(supporter_offx, supporter_offy, supporter_offz);
    Vector3D armbase(armbase_offx, armbase_offy, armbase_offz);
    Vector3D kinect(kinect_offx, kinect_offy, kinect_offz);
    Vector3D cam1394(cam1394_offx, cam1394_offy, cam1394_offz);

    Pose3D elevPose = Pose3D(elevator).translate(0,0,elevHei).rotateY(elevator_lean).translate(0, 0, base_h);
    Pose3D armPose = elevPose;
    armPose.translate(armbase);
    Pose3D supportPose = elevPose;
//    supportPose.translate(supporter);
    supportPose.rotateY(supporter_pitch).rotateX(supporter_roll).rotateZ(supporter_yaw).translate(supporter);
    Pose3D endPose = supportPose;
    endPose.rotateZ(fromDegrees(panAng)).rotateY(fromDegrees(-tiltAng));
    Pose3D kinectPose = endPose;
    kinectPose.translate(kinect).rotateX(kinect_roll).rotateZ(kinect_yaw).rotateY(kinect_pitch);
//    kinectPose.rotateZ(fromDegrees(panAng)).rotateY(fromDegrees(-tiltAng))
//            .rotateX(kinect_roll).rotateZ(kinect_yaw).rotateY(kinect_pitch);
    Pose3D cam1394Pose = endPose;
    cam1394Pose.translate(cam1394).rotateX(cam1394_roll).rotateZ(cam1394_yaw).rotateY(cam1394_pitch);

    // broadcast tf
    // armbase_link
    tf::Transform armTrans;
    armTrans.setOrigin(tf::Vector3(armPose.translation.x, armPose.translation.y,
                                    armPose.translation.z));
    RotationMatrix& am = armPose.rotation;
    armTrans.setBasis(tf::Matrix3x3 (am.c[0].x, am.c[1].x, am.c[2].x,
                                   am.c[0].y, am.c[1].y, am.c[2].y,
                                   am.c[0].z, am.c[1].z, am.c[2].z));
    m_tf->sendTransform(tf::StampedTransform(armTrans, time, g_baseFrame, "armbase_link"));
    // supportor_link
    tf::Transform supTrans;
    supTrans.setOrigin(tf::Vector3(supportPose.translation.x, supportPose.translation.y,
                                    supportPose.translation.z));
    RotationMatrix& sm = supportPose.rotation;
    supTrans.setBasis(tf::Matrix3x3 (sm.c[0].x, sm.c[1].x, sm.c[2].x,
                                   sm.c[0].y, sm.c[1].y, sm.c[2].y,
                                   sm.c[0].z, sm.c[1].z, sm.c[2].z));
    m_tf->sendTransform(tf::StampedTransform(supTrans, time, g_baseFrame, "supportor_link"));
    // openni_camera
    tf::Transform kinectTrans;
    kinectTrans.setOrigin(tf::Vector3(kinectPose.translation.x, kinectPose.translation.y,
                                      kinectPose.translation.z));
    RotationMatrix& km = kinectPose.rotation;
    kinectTrans.setBasis(tf::Matrix3x3 (km.c[0].x, km.c[1].x, km.c[2].x,
                                     km.c[0].y, km.c[1].y, km.c[2].y,
                                     km.c[0].z, km.c[1].z, km.c[2].z));
    m_tf->sendTransform(tf::StampedTransform(kinectTrans, time, g_baseFrame, kinect_frame));

    // camera_1394
    tf::Transform camTrans;
    camTrans.setOrigin(tf::Vector3(cam1394Pose.translation.x, cam1394Pose.translation.y,
                                   cam1394Pose.translation.z));
    RotationMatrix& cm = cam1394Pose.rotation;
    camTrans.setBasis(tf::Matrix3x3 (cm.c[0].x, cm.c[1].x, cm.c[2].x,
                                  cm.c[0].y, cm.c[1].y, cm.c[2].y,
                                  cm.c[0].z, cm.c[1].z, cm.c[2].z));
    m_tf->sendTransform(tf::StampedTransform(camTrans, time, g_baseFrame, cam1394_frame));
}
