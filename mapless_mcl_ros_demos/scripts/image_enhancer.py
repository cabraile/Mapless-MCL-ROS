#!/usr/bin/python3

# ROS
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Image processing
import numpy as np
from skimage.transform import rescale
from skimage.exposure import equalize_hist

class Node:
    
    def __init__(self):
        rospy.init_node("image_enhancer")
        self.init_ros_communication()
        
    def init_ros_communication(self)    -> None:
        self.image_publisher     = rospy.Publisher("/image/enhanced", Image, queue_size=1)
        self.image_subscriber   = rospy.Subscriber("/camera/image", Image, self.image_callback, queue_size=1)
        self.bridge = CvBridge()
    
    def image_array_to_msg(self, image : np.ndarray, frame_id : str, stamp : rospy.Time, encoding : str) -> Image:
        img_out = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        img_out.header.frame_id = frame_id
        img_out.header.stamp = stamp
        return img_out

    def image_callback(self, msg : Image) -> None:
        # Ignore old messages
        stamp = msg.header.stamp
        duration = (rospy.Time.now() - stamp).to_sec()
        if abs(duration) >= 0.1:
            return

        # Convert from message to array
        try:
            image_array_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        # Preprocessing
        image_rgb = image_array_bgr[...,::-1]
        enhanced_rgb =  ( equalize_hist(
            rescale(
                image_rgb,
                scale=0.5, 
                channel_axis=2
             ) 
         ) * 255 ).astype(np.uint8)

        # Publish
        img_out = self.image_array_to_msg(enhanced_rgb, msg.header.frame_id, msg.header.stamp, "rgb8")
        self.image_publisher.publish(img_out)
        
if __name__ == "__main__":
    node = Node()
    rospy.spin()