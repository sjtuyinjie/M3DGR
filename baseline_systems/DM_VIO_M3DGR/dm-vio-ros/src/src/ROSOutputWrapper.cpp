/**
* ROS driver for DM-VIO written by Lukas von Stumberg (http://vision.in.tum.de/dm-vio).
*
* Copyright (c) 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>.
* for more information see <http://vision.in.tum.de/dm-vio>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DM-VIO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DM-VIO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DM-VIO. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ROSOutputWrapper.h"
#include <GTSAMIntegration/PoseTransformationIMU.h>
#include <std_msgs/Int32.h>
#include "dmvio_ros/DMVIOPoseMsg.h"
#include <util/FrameShell.h>
#include <geometry_msgs/PoseStamped.h>

using namespace dmvio;

dmvio::ROSOutputWrapper::ROSOutputWrapper()
        : nh("dmvio")
{
    systemStatePublisher = nh.advertise<std_msgs::Int32>("system_status", 10);
    dmvioPosePublisher = nh.advertise<dmvio_ros::DMVIOPoseMsg>("frame_tracked", 10);
    unscaledPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("unscaled_pose", 10);
    // While we publish the metric pose for convenience we don't recommend using it.
    // The reason is that the scale used for generating it might change over time.
    // Usually it is better to save the trajectory and multiply all of it with the newest scale.
    metricPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("metric_pose", 10);
}


void ROSOutputWrapper::publishTransformDSOToIMU(const TransformDSOToIMU& transformDSOToIMUPassed)
{
    std::unique_lock<std::mutex> lk(mutex);
    transformDSOToIMU = std::make_unique<dmvio::TransformDSOToIMU>(transformDSOToIMUPassed,
                                                                   std::make_shared<bool>(false),
                                                                   std::make_shared<bool>(false),
                                                                   std::make_shared<bool>(false));
    scaleAvailable = lastSystemStatus == SystemStatus::VISUAL_INERTIAL;
    // You could also publish the new scale (and potentially gravity direction) here already if you want to use it as
    // soon as possible. For this simple ROS wrapper I decided to publish it bundled with the newest tracked pose as
    // this is when it is usually needed.
}

void ROSOutputWrapper::publishSystemStatus(dmvio::SystemStatus systemStatus)
{
    std_msgs::Int32 msg;
    msg.data = static_cast<int>(systemStatus);
    systemStatePublisher.publish(msg);
    lastSystemStatus = systemStatus;
}

void setMsgFromSE3(geometry_msgs::Pose& poseMsg, const Sophus::SE3d& pose)
{
    poseMsg.position.x = pose.translation()[0];
    poseMsg.position.y = pose.translation()[1];
    poseMsg.position.z = pose.translation()[2];
    poseMsg.orientation.x = pose.so3().unit_quaternion().x();
    poseMsg.orientation.y = pose.so3().unit_quaternion().y();
    poseMsg.orientation.z = pose.so3().unit_quaternion().z();
    poseMsg.orientation.w = pose.so3().unit_quaternion().w();
}

void ROSOutputWrapper::publishCamPose(dso::FrameShell* frame, dso::CalibHessian* HCalib)
{
    dmvio_ros::DMVIOPoseMsg msg;
    msg.header.stamp = ros::Time(frame->timestamp);
    msg.header.frame_id = "world";

    auto& camToWorld = frame->camToWorld;

    geometry_msgs::Pose& poseMsg = msg.pose;
    setMsgFromSE3(poseMsg, camToWorld);

    // Also publish unscaled pose on its own (e.g. for visualization in Rviz).
    geometry_msgs::PoseStamped unscaledMsg;
    unscaledMsg.header = msg.header;
    unscaledMsg.pose = poseMsg;
    unscaledPosePublisher.publish(unscaledMsg);

    {
        std::unique_lock<std::mutex> lk(mutex);
        if(transformDSOToIMU && scaleAvailable)
        {
            msg.scale = transformDSOToIMU->getScale();

            // Publish scaled pose.
            geometry_msgs::PoseStamped scaledMsg;
            scaledMsg.header = msg.header;

            // Transform to metric imu to world. Note that we need to use the inverse as transformDSOToIMU expects
            // worldToCam as an input!
            Sophus::SE3d imuToWorld(transformDSOToIMU->transformPose(camToWorld.inverse().matrix()));
            setMsgFromSE3(scaledMsg.pose, imuToWorld);

            metricPosePublisher.publish(scaledMsg);
        }else
        {
            msg.scale = std::numeric_limits<double>::quiet_NaN();
            if(transformDSOToIMU) assert(transformDSOToIMU->getScale() == 1.0);
        }

        if(transformDSOToIMU)
        {
            Sophus::SO3d gravityDirection = transformDSOToIMU->getR_dsoW_metricW();
            msg.rotationMetricToDSO.x = gravityDirection.unit_quaternion().x();
            msg.rotationMetricToDSO.y = gravityDirection.unit_quaternion().y();
            msg.rotationMetricToDSO.z = gravityDirection.unit_quaternion().z();
            msg.rotationMetricToDSO.w = gravityDirection.unit_quaternion().w();
            setMsgFromSE3(msg.imuToCam, transformDSOToIMU->getT_cam_imu());
        }
    }

    dmvioPosePublisher.publish(msg);
}

