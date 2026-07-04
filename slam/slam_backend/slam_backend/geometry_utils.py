import numpy as np
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped


def transform_msg_to_matrix(transform) -> np.ndarray:
    """geometry_msgs/Transform -> 4x4 matrix (as returned by a TF lookup)."""
    T = np.eye(4)
    t = transform.translation
    T[0, 3], T[1, 3], T[2, 3] = t.x, t.y, t.z
    q = transform.rotation
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return T


def odom_to_matrix(msg: Odometry) -> np.ndarray:
    T = np.eye(4)
    p = msg.pose.pose.position
    T[0, 3] = p.x
    T[1, 3] = p.y
    T[2, 3] = p.z
    q = msg.pose.pose.orientation
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return T


def matrix_to_odom(T: np.ndarray, stamp, frame_id: str,
                    child_frame_id: str = 'bluerov2/base_link') -> Odometry:
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id

    msg.pose.pose.position.x = float(T[0, 3])
    msg.pose.pose.position.y = float(T[1, 3])
    msg.pose.pose.position.z = float(T[2, 3])

    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    msg.pose.pose.orientation.x = float(q[0])
    msg.pose.pose.orientation.y = float(q[1])
    msg.pose.pose.orientation.z = float(q[2])
    msg.pose.pose.orientation.w = float(q[3])
    return msg


def matrix_to_transform_stamped(T: np.ndarray, stamp, frame_id: str,
                                 child_frame_id: str) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id

    msg.transform.translation.x = float(T[0, 3])
    msg.transform.translation.y = float(T[1, 3])
    msg.transform.translation.z = float(T[2, 3])

    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    msg.transform.rotation.x = float(q[0])
    msg.transform.rotation.y = float(q[1])
    msg.transform.rotation.z = float(q[2])
    msg.transform.rotation.w = float(q[3])
    return msg


def matrix_to_pose_stamped(T: np.ndarray, stamp, frame_id: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(T[0, 3])
    msg.pose.position.y = float(T[1, 3])
    msg.pose.position.z = float(T[2, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])
    return msg


def orthonormalize(R: np.ndarray) -> np.ndarray:
    """Project a near-rotation 3x3 matrix back onto SO(3) via SVD.

    Repeated composition of small floating-point errors (e.g. propagating
    initial-guess poses across many keyframes) can drift a rotation block
    away from strict orthonormality. Cheap to apply defensively before
    handing a matrix to gtsam.Pose3.
    """
    U, _, Vt = np.linalg.svd(R)
    R_ortho = U @ Vt
    if np.linalg.det(R_ortho) < 0:
        U[:, -1] *= -1
        R_ortho = U @ Vt
    return R_ortho
