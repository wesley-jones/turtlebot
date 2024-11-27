import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import zmq
from .config_utils import get_port_from_config

class LidarToLavaNode(Node):
    def __init__(self):
        super().__init__('lidar_to_lava')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            1)
        
        # Set up ZeroMQ context and publisher
        port = get_port_from_config("lidar_sensor")
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.bind(f"tcp://*:{port}")  # Bind to a ZeroMQ port

    def scan_callback(self, msg):
        # Divide the scan data into left, center, and right sectors
        ranges = np.array(msg.ranges)
        left_sector = np.min(ranges[:len(ranges) // 3])  # Left third
        center_sector = np.min(ranges[len(ranges) // 3: 2 * len(ranges) // 3])  # Middle third
        right_sector = np.min(ranges[2 * len(ranges) // 3:])  # Right third

        # Normalize the distances and send them to the SNN input layer
        lidar_input = np.array([left_sector, center_sector, right_sector])
        lidar_input = np.clip(lidar_input, msg.range_min, msg.range_max)  # Clip to sensor range

        # Send the data to Lava SNN over ZeroMQ
        message = ','.join(map(str, lidar_input))  # Convert to a comma-separated string
        self.zmq_socket.send_string(message)  # Send the data
        # self.get_logger().info(f'Lidar input sent to SNN: {lidar_input}')

def main(args=None):
    rclpy.init(args=args)
    node = LidarToLavaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
