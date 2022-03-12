import numpy as np
import tf2_ros

def tf_from_arrays( translation : np.ndarray, rotation : np.ndarray , frame_id : str, child_frame_id : str ) -> tf2_ros.TransformStamped:
    """
    Arguments
    --------
    translation:
        The 3D-flattened-array containing the x,y and z.
    rotation:
        The 4D-flattened-array of the quaternion in the w,x,y,z sequence.
    frame_id:
        The frame id of the parent frame.
    child_frame_id:
        The frame id of the child frame.
    """
    transform = tf2_ros.TransformStamped()
    transform.header.frame_id = frame_id
    transform.child_frame_id = child_frame_id
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.w = rotation[0]
    transform.transform.rotation.x = rotation[1]
    transform.transform.rotation.y = rotation[2]
    transform.transform.rotation.z = rotation[3]
    return transform
