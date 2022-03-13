#!/usr/bin/python3
"""Changes the frame id of the GNSS message and changes the covariance for
metric coordinates.
"""
import numpy as np
import rospy
from sensor_msgs.msg import NavSatFix

class Node:

    def __init__(self):
        rospy.init_node("carina_demo_gps_fixer")

        self.navsat_fix_subscriber = rospy.Subscriber("/carina/driver/septentrio/fix", NavSatFix, self.subscriber_navsat_fix, queue_size=10)
        self.navsat_fix_publisher = rospy.Publisher("/carina/driver/septentrio/fix_corrected", NavSatFix, queue_size=10)

        self.default_confidence_xy = 0.04 # meters - RTK
        self.default_confidence_altitude = 10.0 # meters

    def subscriber_navsat_fix(self, msg : NavSatFix):

        # Prepare message
        out_msg = NavSatFix()
        out_msg.header = msg.header
        out_msg.header.frame_id = "gps"
        out_msg.status = msg.status
        out_msg.latitude = msg.latitude
        out_msg.longitude = msg.longitude
        out_msg.altitude = msg.altitude
        xy_variance = ( self.default_confidence_xy / 3.0 ) ** 2.0
        altitude_variance = ( self.default_confidence_altitude / 3.0 ) ** 2.0
        out_msg.position_covariance = np.diag([xy_variance, xy_variance, altitude_variance]).flatten()
        out_msg.position_covariance_type = msg.position_covariance_type

        # Publish
        self.navsat_fix_publisher.publish(out_msg)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    node = Node()
    node.spin()
