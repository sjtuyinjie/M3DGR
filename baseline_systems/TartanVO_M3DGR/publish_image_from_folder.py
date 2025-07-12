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
from os import listdir
from os.path import isfile
import time

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class PubImgFolder(object):
    def __init__(self):

        image_dir = rospy.get_param('~img_dir', 'data/M3DGR/visual/indoor/dynamic_person_1')
        pose_file = rospy.get_param('~pose_file', '')#data/EuRoC_V102/pose_left.txt

        self.cv_bridge = CvBridge()
        self.img_pub = rospy.Publisher("rgb_image", Image, queue_size=10)
        self.caminfo_pub = rospy.Publisher("cam_info", CameraInfo, queue_size=10)
        self.scale_pub = rospy.Publisher("vo_scale", Float32, queue_size=10)

        files = listdir(image_dir)
        self.rgbfiles = [(image_dir +'/'+ ff) for ff in files if (ff.endswith('.png') or ff.endswith('.jpg'))]
        self.rgbfiles.sort()
        self.image_dir = image_dir

        print('Find {} image files in {}'.format(len(self.rgbfiles), image_dir))
        self.imgind = 0

        if isfile(pose_file):
            self.poselist = np.loadtxt(pose_file)
            if len(self.poselist) != len(self.rgbfiles):
                print('Posefile {} does not have the same length with the rgb images'.format(pose_file))
                self.poselist=None
        else:
            self.poselist=None

    def caminfo_publish(self):
        caminfo = CameraInfo()

        #m3dgr
        caminfo.width   = 640
        caminfo.height  = 480
        caminfo.K[0]    = 607.79772949218
        caminfo.K[4]    = 607.83526613281
        caminfo.K[2]    = 328.79772949218
        caminfo.K[5]    = 245.53321838378 

        # image info for EuRoC
        # caminfo.width   = 752
        # caminfo.height  = 480
        # caminfo.K[0]    = 458.6539916992
        # caminfo.K[4]    = 457.2959899902
        # caminfo.K[2]    = 367.2149963379
        # caminfo.K[5]    = 248.3750000000

        # # image info for KTIIT_10
        # caminfo.width   = 1226
        # caminfo.height  = 370
        # caminfo.K[0]    = 707.0912
        # caminfo.K[4]    = 707.0912
        # caminfo.K[2]    = 601.8873
        # caminfo.K[5]    = 183.1104

        self.caminfo_pub.publish(caminfo)

    def img_publish(self):
        if self.imgind >= len(self.rgbfiles):
            return False
        # publish GT scale from the posefile
        if self.poselist is not None and self.imgind > 0:
            trans = self.poselist[self.imgind][:3] - self.poselist[self.imgind-1][:3]
            dist = np.linalg.norm(trans) 
            scale_msg = Float32()
            scale_msg.data = dist
            self.scale_pub.publish(scale_msg)
        # publish image
        img = cv2.imread(self.rgbfiles[self.imgind])
        if len(img.shape)==2:
            img = np.stack([img, img, img])
        img_msg = self.cv_bridge.cv2_to_imgmsg(img, "bgr8")
        self.img_pub.publish(img_msg)
        self.imgind += 1
        return True

if __name__ == '__main__':
    rospy.init_node("img_folder_pub", log_level=rospy.INFO)
    node = PubImgFolder()
    node.caminfo_publish()
    time.sleep(0.1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.caminfo_publish()
        ret = node.img_publish()
        rate.sleep()
        if not ret:
            break
