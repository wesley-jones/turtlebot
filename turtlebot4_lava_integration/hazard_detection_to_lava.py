import zmq
import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import HazardDetectionVector
from .config_utils import get_port_from_config

class HazardSensorToLavaNode(Node):
    def __init__(self):
        super().__init__('hazard_sensor_to_snn')
        self.subscription = self.create_subscription(
            HazardDetectionVector,
            '/hazard_detection',
            self.listener_callback,
            1
        )
        # ZeroMQ setup
        port = get_port_from_config("hazard_sensor")
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind("tcp://*:{port}")  # Dedicated port for hazard sensor

    def listener_callback(self, msg):
        for detection in msg.detections:
            match detection.type:
                case 2:  # CLIFF constant
                    self.socket.send_string("cliff")
                case 1:  # BUMP constant
                    self.socket.send_string("bump")
                case _:  # Default case for unhandled hazard types
                    self.socket.send_string("unknown")


def main(args=None):
    rclpy.init(args=args)
    node = HazardSensorToLavaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
