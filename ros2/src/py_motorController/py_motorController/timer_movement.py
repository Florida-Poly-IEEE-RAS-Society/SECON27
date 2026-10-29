import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class timerMovement(Node):
    rclpy.init()

def entrypoint():
    rclpy.init()

    node = Node("cmd_vel_publisher")
    publisher = node.create_publisher(Twist, "cmd_vel", 10)

    def publish_velocity():
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0

        publisher.publish(msg)

    publish_Velocity()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
