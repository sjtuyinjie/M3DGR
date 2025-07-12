#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, Wenshan Wang, Yaoyu Hu,  CMU
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of CMU nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from cv_bridge import CvBridge

from Datasets.utils import ToTensor, Compose, CropCenter, DownscaleFlow, make_intrinsics_layer
from Datasets.transformation import se2SE, SO2quat
from TartanVO import TartanVO
import time

class TartanVONode(object):
    def __init__(self):

        model_name = rospy.get_param('~model_name', 'tartanvo_1914.pkl')
        w = rospy.get_param('~image_width', 640)
        h = rospy.get_param('~image_height', 480)
        fx = rospy.get_param('~focal_x', 320.0)
        fy = rospy.get_param('~focal_y', 320.0)
        ox = rospy.get_param('~center_x', 320.0)
        oy = rospy.get_param('~center_y', 240.0)
        self.cam_intrinsics = [w, h, fx, fy, ox, oy]

        self.cv_bridge = CvBridge()
        self.transform = Compose([CropCenter((448, 640)), DownscaleFlow(), ToTensor()])#, Normalize(mean=[0., 0., 0.],std=[1., 1., 1.])])
        self.intrinsic = make_intrinsics_layer(w, h, fx, fy, ox, oy)
        self.tartanvo = TartanVO(model_name)

        self.pose_pub = rospy.Publisher("tartanvo_pose", PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher("tartanvo_odom", Odometry, queue_size=10)
        rospy.Subscriber('rgb_image', Image, self.handle_img)
        rospy.Subscriber('cam_info', CameraInfo, self.handle_caminfo)
        rospy.Subscriber('vo_scale', Float32, self.handle_scale)

        self.last_img = None
        self.pose = np.matrix(np.eye(4,4))
        self.scale = 1.0
        self.timestamp = 1732437229.406543  # 初始时间戳
        self.time_increment = 1.0 / 30.0  # 30Hz 的时间增量

    def handle_caminfo(self, msg):
        w = msg.width
        h = msg.height
        fx = msg.K[0]
        fy = msg.K[4]
        ox = msg.K[2]
        oy = msg.K[5]
        new_intrinsics = [w, h, fx, fy, ox, oy]
        change = [xx!=yy for xx,yy in zip(new_intrinsics, self.cam_intrinsics)]
        if True in change:
            self.intrinsic = make_intrinsics_layer(w, h, fx, fy, ox, oy)
            self.cam_intrinsics = [w, h, fx, fy, ox, oy]
            print('Camera intrinsics updated..')

    def handle_scale(self, msg):
        self.scale = msg.data

    def handle_img(self, msg):
        starttime = time.time()
        image_np = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

        if image_np.shape[0] != self.intrinsic.shape[0] or image_np.shape[1] != self.intrinsic.shape[1]:
            print('The intrinsic parameter does not match the image parameter!')
            return

        if self.last_img is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = 'map'
            sample = {'img1': self.last_img, 
                      'img2': image_np, 
                      'intrinsic': self.intrinsic
                      }
            sample = self.transform(sample)
            sample['img1'] = sample['img1'][None] # increase the dimension
            sample['img2'] = sample['img2'][None]
            sample['intrinsic'] = sample['intrinsic'][None]

            motion, _ = self.tartanvo.test_batch(sample)
            motion = motion[0]
            # adjust the scale if available
            if self.scale!=1:
                trans = motion[:3]
                trans = trans / np.linalg.norm(trans) * self.scale
                motion[:3] = trans
                print(self.scale)

            motion_mat = se2SE(motion)
            self.pose = self.pose * motion_mat
            quat = SO2quat(self.pose[0:3,0:3])

            pose_msg.pose.position.x = self.pose[0,3]
            pose_msg.pose.position.y = self.pose[1,3]
            pose_msg.pose.position.z = self.pose[2,3]
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            self.pose_pub.publish(pose_msg)     

            odom_msg = Odometry()
            odom_msg.header = pose_msg.header
            odom_msg.pose.pose = pose_msg.pose

            self.odom_pub.publish(odom_msg)  

            # 将位姿数据格式化为 TUM 格式
            timestamp = self.timestamp
            self.timestamp += self.time_increment  # 递增时间戳
            print("Current ROS time as timestamp: ", timestamp)
            tum_format_pose = "{:.6f} {} {} {} {} {} {} {}".format(
            timestamp,
            self.pose[0, 3], self.pose[1, 3], self.pose[2, 3],  # 平移部分
            quat[0], quat[1], quat[2], quat[3]  # 旋转部分（四元数）
            )

            # 将格式化的位姿数据写入 .txt 文件
            with open('./traj.txt', 'a') as f:
                f.write(tum_format_pose + '\n')    

        self.last_img = image_np.copy()
        print("    call back time: {}:".format(time.time()-starttime))

if __name__ == '__main__':
    rospy.init_node("tartanvo_node", log_level=rospy.INFO)
    node = TartanVONode()
    rospy.spin()
