#!/usr/bin/python3
import os
import sys
import rospy
import numpy as np

import pandas as pd

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def euclidean_distance(P : Odometry, Q : Odometry) -> float:
    Px = P.pose.pose.position.x
    Py = P.pose.pose.position.y
    Qx = Q.pose.pose.position.x
    Qy = Q.pose.pose.position.y
    return ( (Px - Qx) ** 2 + (Py - Qy) ** 2 ) ** 0.5

def mahalanobis_distance(ref : Odometry, comp : Odometry) -> float:
    Rx = ref.pose.pose.position.x
    Ry = ref.pose.pose.position.y
    Cx = comp.pose.pose.position.x
    Cy = comp.pose.pose.position.y
    Q = np.array( ref.pose.covariance ).reshape(6,6)[:2,:2]
    diff = np.array([[Cx-Rx],[Cy-Ry]])
    distance = float( np.sqrt( diff.T @ np.linalg.inv(Q) @ diff ) )
    return distance

def angular_difference(P1 : Odometry, P2 : Odometry) -> float:
    # Retrieve the orientation quaternions
    q1 = ( P1.pose.pose.orientation.w, P1.pose.pose.orientation.x, P1.pose.pose.orientation.y, P1.pose.pose.orientation.z )
    q2 = ( P2.pose.pose.orientation.w, P2.pose.pose.orientation.x, P2.pose.pose.orientation.y, P2.pose.pose.orientation.z )
    # Convert quaternions to RPY
    _,_, angle1 = euler_from_quaternion(q1)
    _,_, angle2 = euler_from_quaternion(q2)
    # Unit orientation vectors
    uv1 = np.array( [ np.cos(angle1), np.sin(angle1) ] )
    uv2 = np.array( [ np.cos(angle2), np.sin(angle2) ] )
    # Angle between orientation vectors
    angle_diff = np.arccos( uv1 @ uv2 )
    return angle_diff

class Node:

    def __init__(self) :
        rospy.init_node("groundtruth_comparison")
        self.output_path = os.path.abspath( rospy.get_param("~output_path") )
        self.subscriber_groundtruth     = rospy.Subscriber("/fusion/groundtruth", Odometry, self.callback, callback_args="/fusion/groundtruth", queue_size=1)
        self.subscriber_gps_only        = rospy.Subscriber("/navsat/odometry",    Odometry, self.callback, callback_args="/navsat/odometry", queue_size=1)
        self.subscriber_odometry_only   = rospy.Subscriber("/fusion/local",       Odometry, self.callback, callback_args="/fusion/local", queue_size=1)
        self.subscriber_drmcl_fused    = rospy.Subscriber("/fusion/global",       Odometry, self.callback, callback_args="/fusion/global", queue_size=1)
        self.subscriber_drmcl_raw      = rospy.Subscriber("/mcl/drmcl/pose",      Odometry, self.callback, callback_args="/mcl/drmcl/pose", queue_size=1)
        
        # Map each topic to each category - simplifies the trouble of searching
        self.topic_to_category_mapper = {
            "/fusion/groundtruth": "groundtruth", 
            "/navsat/odometry" :   "gps_only",  
            "/fusion/local" :      "odometry_only",
            "/fusion/global":      "drmcl_fused",
            "/mcl/drmcl/pose":     "drmcl_raw",
        }

        self.entries = []
        # Stores the last message received for each comparison category
        self.clear_messages()

    def clear_messages ( self ) -> None:
        self.messages = {
            "groundtruth" : None, "gps_only" : None, "drmcl_fused" : None, "odometry_only" : None, "drmcl_raw" : None
        }

    def spin( self ) -> None:
        rate = rospy.Rate(50)
        try:
            while not rospy.is_shutdown():
                self.compare()
                self.clear_messages()
                rate.sleep()
        finally:
            self.store()

    def callback(self, msg : Odometry, topic_name : str) -> None:
        category = self.topic_to_category_mapper[topic_name]
        self.messages[category] = msg

    def compare(self) -> None:

        for category in ["gps_only", "drmcl_fused", "drmcl_raw"]:
            groundtruth = self.messages["groundtruth"]
            # Check if received groundtruth
            if groundtruth is None:
                return
            message = self.messages[category]
            # Check if received the first message of the category
            if message is None:
                continue

            # Compute Euclidean distance
            euc_distance = euclidean_distance(groundtruth, message)

            # Compute Mahalanobis distance
            mah_distance = mahalanobis_distance(groundtruth, message)

            # Compute angular difference
            ang_difference = angular_difference(groundtruth, message)
            
            # Store metrics and more values for later comparison
            entry = { 
                "timestamp" : message.header.stamp,
                "category" : category ,
                "euclidean_distance" : euc_distance,
                "mahalanobis_distance" : mah_distance,
                "angular_difference" : ang_difference,
                "groundtruth_x" : groundtruth.pose.pose.position.x,
                "groundtruth_y" : groundtruth.pose.pose.position.y, 
                "estimated_x" : message.pose.pose.position.x,
                "estimated_y" : message.pose.pose.position.x,
            }
            self.entries.append(entry)

    def store(self) -> None:
        df = pd.DataFrame(self.entries)
        df.to_csv(self.output_path)

def main() -> int:
    node = Node()
    node.spin()
    return 0

if __name__ == "__main__":
    sys.exit(main())