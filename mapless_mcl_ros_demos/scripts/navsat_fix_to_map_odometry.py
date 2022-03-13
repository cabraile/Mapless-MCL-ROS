#!/usr/bin/python3
"""Projects the GNSS messages to the map frame published by the filter.

The output Odometry message is used as input for the groundtruth EKF.
"""
import os
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from mapless_mcl.trajectory import Trajectory
import utm

class Node:

    def __init__(self):
        rospy.init_node("navsat_fix_to_map_odometry")
        # Load the trajectory origin
        trajectory_path = os.path.abspath( rospy.get_param("~trajectory_path") )
        self.origin = Trajectory(trajectory_path).get_origin()

        self.navsat_fix_subscriber = rospy.Subscriber("/navsat/fix", NavSatFix, self.subscriber_navsat_fix, queue_size=10)
        self.odom_publisher = rospy.Publisher("/navsat/odometry", Odometry, queue_size=10)

    def subscriber_navsat_fix(self, msg : NavSatFix):
        easting, northing,_,_ = utm.from_latlon(msg.latitude, msg.longitude)
        x = easting - self.origin.x
        y = northing - self.origin.y
        pose_covariance = np.diag((6 * [1e9]))
        pose_covariance[:3,:3] = np.array(msg.position_covariance).reshape(3,3)

        # Prepare message
        out_msg = Odometry()
        out_msg.header.stamp = rospy.Time.now()
        out_msg.header.frame_id = "map"
        out_msg.child_frame_id = msg.header.frame_id
        out_msg.pose.covariance = pose_covariance.flatten()
        out_msg.pose.pose.position.x = x
        out_msg.pose.pose.position.y = y
        out_msg.pose.pose.position.z = 0.0#msg.altitude
        out_msg.pose.pose.orientation.w = 1.0
        out_msg.pose.pose.orientation.x = 0.0
        out_msg.pose.pose.orientation.y = 0.0
        out_msg.pose.pose.orientation.z = 0.0
        
        # Publish
        self.odom_publisher.publish(out_msg)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    node = Node()
    node.spin()
