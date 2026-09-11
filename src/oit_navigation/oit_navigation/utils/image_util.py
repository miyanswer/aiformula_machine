"""
image_util.py - Robust and Lightweight OpenCV <-> ROS 2 Image Converter
Avoids OpenCV 5 / NumPy 2 / cv_bridge binary mapping compatibility issues.
"""

from typing import Optional
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge


def cv2_to_imgmsg(
    cv_img: np.ndarray,
    encoding: str = 'bgr8',
    frame_id: str = '',
    stamp=None
) -> Image:
    """Fast and robust pure-python conversion from numpy ndarray to sensor_msgs/Image."""
    msg = Image()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = cv_img.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0

    if encoding == 'mono8':
        msg.step = int(msg.width)
    elif encoding in ('bgr8', 'rgb8'):
        msg.step = int(msg.width * 3)
    elif encoding in ('bgra8', 'rgba8'):
        msg.step = int(msg.width * 4)
    else:
        # Fallback to standard CvBridge
        try:
            return CvBridge().cv2_to_imgmsg(cv_img, encoding=encoding)
        except Exception:
            msg.step = int(msg.width * (cv_img.shape[2] if cv_img.ndim > 2 else 1))

    msg.data = cv_img.tobytes()
    return msg


def imgmsg_to_cv2(msg: Image, desired_encoding: str = 'bgr8') -> np.ndarray:
    """Fast and robust conversion from sensor_msgs/Image to numpy ndarray."""
    try:
        if msg.encoding in ('bgr8', 'rgb8'):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            if msg.encoding == 'rgb8' and desired_encoding == 'bgr8':
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8' and desired_encoding == 'rgb8':
                return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            return img.copy()
        elif msg.encoding == 'mono8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            if desired_encoding == 'bgr8':
                return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            return img.copy()
    except Exception:
        pass

    # Fallback
    return CvBridge().imgmsg_to_cv2(msg, desired_encoding=desired_encoding)
