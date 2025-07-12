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

#ifndef DMVIO_ROS_ROSOUTPUTWRAPPER_H
#define DMVIO_ROS_ROSOUTPUTWRAPPER_H

#include <IOWrapper/Output3DWrapper.h>
#include <ros/ros.h>
#include <mutex>

namespace dmvio
{

// We publish 3 topics by default:
// dmvio/frame_tracked: DMVIOPoseMsg
// dmvio/unscaled_pose: PoseStamped
// dmvio/metric_poses: PoseStamped
// For more details on these see the README.md file.
class ROSOutputWrapper : public dso::IOWrap::Output3DWrapper
{
public:
    ROSOutputWrapper();

    /*
     * Usage:
     * Called once after each keyframe is optimized and passes the new transformation from DSO frame (worldToCam in
     * DSO scale) to metric frame (imuToWorld in metric scale).
     * Use transformPose of the passed object to transform poses between the frames.
     * Note that the object should not be used any more after the method returns.
     * The caller can create copy however , preferable with the following constructor (as otherwise the shared_ptrs will be kept).
     * TransformDSOToIMU(TransformDSOToIMU& other, std::shared_ptr<bool> optScale, std::shared_ptr<bool> optGravity, std::shared_ptr<bool> optT_cam_imu);
     */
    virtual void publishTransformDSOToIMU(const dmvio::TransformDSOToIMU& transformDSOToIMU) override;

    /*
     * Usage:
     * Called every time the status of the system changes.
     */
    virtual void publishSystemStatus(dmvio::SystemStatus systemStatus) override;


    /* Usage:
     * Called once for each tracked frame, with the real-time, low-delay frame pose.
     *
     * Calling:
     * Always called, no overhead if not used.
     */
    virtual void publishCamPose(dso::FrameShell* frame, dso::CalibHessian* HCalib) override;

    // In case you want to additionally publish pointclouds or keyframe poses you need to override Output3DWrapper::publishKeyframes

private:
    ros::NodeHandle nh;
    ros::Publisher dmvioPosePublisher, systemStatePublisher, unscaledPosePublisher, metricPosePublisher;

    // Protects transformDSOToIMU.
    std::mutex mutex;

    std::unique_ptr<dmvio::TransformDSOToIMU> transformDSOToIMU;
    bool scaleAvailable = false; // True iff transformDSOToIMU contains a valid scale.
    std::atomic<dmvio::SystemStatus> lastSystemStatus;
};


}
#endif //DMVIO_ROS_ROSOUTPUTWRAPPER_H
