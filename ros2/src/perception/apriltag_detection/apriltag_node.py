"""ROS 2 node that detects AprilTags and publishes the decoded ids."""

import json

from apriltag_detection.detector import StableTagVote, TagDetector
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger


class AprilTagNode(Node):
    """Detect AprilTags in camera images."""

    def __init__(self):
        """Declare parameters, build the detector, and wire topics."""
        super().__init__('apriltag_detector')
        params = {
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'family': 'tagStandard41h12',
            # Detector corners sit on the black border: 0.068 m * 5/9 cells.
            'tag_size': 0.0378,
            'valid_ids': [1, 2, 3],
            'max_hamming': 0,
            'min_margin': 20.0,
            'decimate': 1.0,
            'threads': 2,
            'confirm_frames': 5,
            # Keep the first confirmed id (the pre-round scan) so the pit tags
            # seen during the race cannot change it. /apriltag/reset clears it.
            'lock_on_confirm': True,
            'publish_debug_image': False,
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in params}

        self.detector = TagDetector(
            p['family'], p['tag_size'], list(p['valid_ids']), p['max_hamming'],
            p['min_margin'], threads=p['threads'], decimate=p['decimate'])
        self.vote = StableTagVote(p['confirm_frames'], p['lock_on_confirm'])
        self.debug = p['publish_debug_image']
        self.bridge = CvBridge()
        self.camera_k = None
        self.pit_spot = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.id_pub = self.create_publisher(Int32, '/apriltag/id', 10)
        self.det_pub = self.create_publisher(String, '/apriltag/detections', 10)
        self.spot_pub = self.create_publisher(Int32, '/apriltag/pit_spot', latched)
        self.debug_pub = (self.create_publisher(Image, '/apriltag/debug_image', 1)
                          if self.debug else None)
        self.create_subscription(CameraInfo, p['camera_info_topic'],
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, p['image_topic'], self.on_image,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, '/apriltag/reset', self.on_reset)

    def on_reset(self, request, response):
        """Forget the confirmed pit spot so the next scan can set it again."""
        self.vote.reset()
        self.pit_spot = None
        self.get_logger().info('Pit spot cleared; waiting for a new scan')
        response.success = True
        return response

    def on_info(self, msg):
        """Cache the pinhole intrinsics."""
        self.camera_k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_image(self, msg):
        """Detect tags in one frame and publish the results."""
        gray = self._to_mono8(msg)
        tags = self.detector.detect(gray, self.camera_k)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.det_pub.publish(String(data=json.dumps({
            'stamp': stamp,
            'frame_id': msg.header.frame_id,
            'detections': [self._to_dict(t) for t in tags],
        })))

        best = tags[0].id if tags else None
        if best is not None:
            self.id_pub.publish(Int32(data=best))
        confirmed = self.vote.update(best)
        if confirmed is not None and confirmed != self.pit_spot:
            self.pit_spot = confirmed
            self.spot_pub.publish(Int32(data=confirmed))
            self.get_logger().info(f'Pit spot confirmed: tag {confirmed}')

        if self.debug_pub is not None:
            self._publish_debug(gray, tags, msg.header)

    def _to_mono8(self, msg):
        """Convert any supported image encoding to a mono8 array."""
        if msg.encoding == 'mono8':
            return self.bridge.imgmsg_to_cv2(msg, 'mono8')
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def _to_dict(tag):
        pose = None
        if tag.pose is not None:
            pose = {
                't': np.asarray(tag.pose['t']).ravel().tolist(),
                'R': np.asarray(tag.pose['R']).tolist(),
                'error': float(tag.pose['error']),
            }
        return {
            'id': tag.id,
            'hamming': tag.hamming,
            'margin': tag.margin,
            'center': list(tag.center),
            'corners': tag.corners.tolist(),
            'pose': pose,
        }

    def _publish_debug(self, gray, tags, header):
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for t in tags:
            quad = t.corners.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(img, [quad], True, (0, 255, 0), 2)
            cv2.putText(img, str(t.id), (int(t.center[0]), int(t.center[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        out = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        out.header = header
        self.debug_pub.publish(out)


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = AprilTagNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
