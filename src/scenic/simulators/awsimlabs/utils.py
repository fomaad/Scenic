import numpy as np
import math
from geometry_msgs.msg import Quaternion

def distance_point_to_segment_2d(px, py, x1, y1, x2, y2):
    """
    Distance between point (px, py) and its projection on the segment (x1, y1)-(x2, y2).
    Also returns the projection_inside_segment flag.
    When the projection point falls outside the segment, 
    return the distance from the point to either starting or ending points of the segment, which is closer
    and the flag is set to False
    """
    line = np.array([x2 - x1, y2 - y1])
    if np.allclose(line, 0):
        return np.hypot(px - x1, py - y1)
    t = np.dot([px - x1, py - y1], line) / np.dot(line, line)
    projection_inside_segment = t>=0 and t<=1
    t = max(0, min(1, t))
    proj = np.array([x1, y1]) + t * line
    return np.linalg.norm([px - proj[0], py - proj[1]]), projection_inside_segment

def distance_point_to_segment_3d(P, A, B):
    """
    See function distance_point_to_segment_2d
    """
    p = np.array(P)
    a = np.array(A)
    b = np.array(B)
    ab = b - a
    if np.allclose(ab, 0):
        return np.linalg.norm(p - a)
    
    t = np.dot(p - a, ab) / np.dot(ab, ab)
    projection_inside_segment = t>=0 and t<=1
    t = np.clip(t, 0, 1)
    proj = a + t * ab
    return np.linalg.norm(p - proj), projection_inside_segment

def project_point_to_line_3d(P, A, B):
    p = np.array(P)
    a = np.array(A)
    b = np.array(B)
    ab = b - a
    if np.allclose(ab, 0):
        return np.linalg.norm(p - a)
    
    t = np.dot(p - a, ab) / np.dot(ab, ab)
    projection_inside_segment = t>=0 and t<=1
    proj = a + t * ab
    return proj, projection_inside_segment

def yaw_to_quaternion(yaw):
    """Convert a yaw angle (in radians) into a ROS2 Quaternion message."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def direction_vector_to_quaternion(dx,dy):
    """Return quaternion aligned to the given 2D direction vector (dx, dy)."""
    yaw = math.atan2(dy, dx)  # direction angle in radians
    return yaw_to_quaternion(yaw)