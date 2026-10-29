import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class imageSub(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.listener_callback, 10)
        self.subscription

        self.br = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info('Receiving Video Frame!!')
        current_frame = self.br.imgmsg_to_cv2(data,desired_encoding="passthrough")
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

def entrypoint(args=None):
    rclpy.init(args=args)
    node = imageSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
