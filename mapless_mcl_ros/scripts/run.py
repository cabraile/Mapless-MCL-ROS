#!/usr/bin/python3
import os
import sys

import numpy as np

import rospy
import ros_numpy
import tf2_ros

from shapely.geometry   import Point
from geometry_msgs.msg  import PoseWithCovarianceStamped
from sensor_msgs.msg    import PointCloud2
from nav_msgs.msg       import Odometry

from mapless_mcl.drmcl      import DRMCL
from mapless_mcl.trajectory import Trajectory
from mapless_mcl.hlmap      import HLMap

from helpers.convert import tf_from_arrays

class DRNode:

    def spin(self) -> None:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()
    
    # INIT
    #==========================================================================

    def __init__(self):
        rospy.init_node("mapless_mcl_ros_runner")
        self.load_args()
        self.init_communication()
        self.setup()

    def load_args(self) -> None:
        self.n_init_particles   = rospy.get_param("~particles")
        self.frame              = rospy.get_param("~frame", "base_footprint")
        self.flag_publish_map_to_frame_tf    = rospy.get_param("~publish_map_to_frame_tf", False)
        self.flag_publish_utm_to_map_tf    = rospy.get_param("~publish_utm_to_map_tf", False)
        self.path_to_map        = os.path.abspath( rospy.get_param("~map_path") )
        self.path_to_trajectory = os.path.abspath( rospy.get_param("~trajectory_path") ) # TODO: pass as a service

    def init_communication(self) -> None:
        self.offset_subscriber  = rospy.Subscriber("/odometry/offset",PoseWithCovarianceStamped, self.offset_callback)
        self.particles_publisher= rospy.Publisher("/mcl/drmcl/particles", PointCloud2, queue_size=10)
        self.state_publisher    = rospy.Publisher("/mcl/drmcl/pose", Odometry, queue_size=1)
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()

    def setup(self) -> None:        
        # Load the map
        self.hlmap = HLMap()
        self.hlmap.load(self.path_to_map)
        
        # Load the initial trajectory
        self.trajectory = Trajectory(self.path_to_trajectory)
        self.set_origin(self.trajectory.get_origin())

        # Starts the filter
        self.mcl = DRMCL()
        self.mcl.sample(
            n_particles = self.n_init_particles, 
            trajectory_id = self.trajectory.id
        )

    def set_origin(self, origin : Point) -> None:
        self.origin = origin

    #==========================================================================

    # CALLBACKS
    #==========================================================================

    def offset_callback(self, msg : PoseWithCovarianceStamped) -> None:
        # TODO: perform checking for frame id
        translation = msg.pose.pose.position
        covariance = np.array( msg.pose.covariance ).reshape(6,6)
        # TODO: use full translation and rotation as input
        control_cmd = np.array( ( translation.x, translation.y ) )
        covariance = np.diag([1.0,1.0])
        self.mcl.predict(control_cmd, covariance)

    #==========================================================================

    # PUBLISHERS
    #==========================================================================

    def publish(self) -> None:

        # Map origin
        origin = self.origin

        # Retrieve estimation
        mean_list, covariance_list = self.mcl.get_position([self.trajectory]) # TODO: provide more trajectories
        mean = mean_list[0] # TODO: iterate for each trajectory
        covariance = covariance_list[0]

        self.publish_state(mean, covariance, origin)
        self.publish_particles(origin)
        self.publish_tf(mean, origin)

    def publish_state(self, mean : Point, covariance : np.ndarray, origin : Point) -> None:

        msg = Odometry()
        # Fill metadata
        msg.header.frame_id = "map"
        msg.child_frame_id = self.frame
        msg.header.stamp = rospy.Time.now()
        # Fill position
        msg.pose.pose.position.x = mean.x - origin.x
        msg.pose.pose.position.y = mean.y - origin.y
        msg.pose.pose.position.z = 0.0 # TODO
        # Fill orientation
        msg.pose.pose.orientation.w = 1.0
        msg.pose.pose.orientation.x = 0.
        msg.pose.pose.orientation.y = 0.
        msg.pose.pose.orientation.z = 0.
        # Fill covariance
        out_covariance = np.zeros((6,6), dtype=float)
        out_covariance[:2,:2] = covariance[:2,:2]
        msg.pose.covariance = out_covariance.flatten()
        # Publish!
        self.state_publisher.publish(msg)
        
    def publish_tf(self, mean : Point, origin : Point) -> None:

        if self.flag_publish_utm_to_map_tf:
            # REPLACE THIS BY A STATIC BROADCASTER
            # Publish utm->map transform
            translation = np.array([origin.x, origin.y, 0])
            rotation = np.array([1.0, 0.0, 0.0, 0.0])
            tf_from_utm_to_map = tf_from_arrays(translation, rotation, "utm", "map")
            tf_from_utm_to_map.header.stamp = rospy.Time.now()
            self.transform_broadcaster.sendTransform(tf_from_utm_to_map)

        # Publish map->frame transform
        if self.flag_publish_map_to_frame_tf:
            translation = np.array([ mean.x - origin.x, mean.y - origin.y, 0.0 ])
            rotation = rotation # TODO: fill with a true rotation
            rospy.loginfo(mean)
            tf_from_map_to_frame = tf_from_arrays(translation, rotation, "map", self.frame)
            tf_from_map_to_frame.header.stamp = rospy.Time.now()
            self.transform_broadcaster.sendTransform(tf_from_map_to_frame)
        
    def publish_particles(self, origin : Point) -> None:
        points = self.mcl.get_particles([self.trajectory])
        
        # The particles' point cloud.
        point_cloud_array = np.zeros( 
            len(points),
            dtype=[
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32)
            ]
        )
        point_cloud_array["x"] = [ p.x - origin.x for p in points ]
        point_cloud_array["y"] = [ p.y - origin.y for p in points ]
        point_cloud_array["z"] = np.zeros(len(points))
        
        cloud_msg = ros_numpy.msgify(ros_numpy.numpy_msg(PointCloud2), point_cloud_array)

        # The particles are referenced based on the map origin
        cloud_msg.header.frame_id = "map"
        self.particles_publisher.publish(cloud_msg)
    
    #==========================================================================

def main() -> int:
    node = DRNode()
    node.spin()
    return 0

if __name__ == "__main__":
    sys.exit(main())