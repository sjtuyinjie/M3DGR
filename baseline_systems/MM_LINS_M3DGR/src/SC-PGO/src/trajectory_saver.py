#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import sys

file = None
file_name = "path_data.txt" 

def path_callback(msg):
    global file
    poses = msg.poses

    # Open the file for writing (overwrite)
    with open(file_name, "w") as file:
        for pose in poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            qx = pose.pose.orientation.x
            qy = pose.pose.orientation.y
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w
            timestamp = pose.header.stamp.to_sec()

            file.write("%f %f %f %f %f %f %f %f\n" % (timestamp, x, y, z, qx, qy, qz, qw))

def main():
    global file
    rospy.init_node("path_saver")
    path_topic = rospy.get_param("~path_topic", "/aft_pgo_path")
    file_name = rospy.get_param("~file_name", "path_data.txt")

    rospy.Subscriber(path_topic, Path, path_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
