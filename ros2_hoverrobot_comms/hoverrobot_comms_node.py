import rclpy
from rclpy.node import Node 
from std_msgs.msg import String # TODO: reemplazar por el tipo de message especifico
from ros2_hoverrobot_comms.hoverrobot_comms import HoverRobotComms



SERVER_IP = '192.168.0.101'
SERVER_PORT = 8080
RECONNECT_DELAY = 5  # segundos entre reintentos

class HoverRobotCommsNode(Node):
    def __init__(self):
        super().__init__('hoverrobot_comms_node')

        self.publishers_ = self.create_publisher(String, 'hoverrobor_dynamic_data', 10)
        self.hoverRobotComms = HoverRobotComms(SERVER_IP, SERVER_PORT, RECONNECT_DELAY)




def main(args=None):
    rclpy.init(args=args)
    node = HoverRobotCommsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()