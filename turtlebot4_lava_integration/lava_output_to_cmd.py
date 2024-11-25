import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import zmq

class LavaOutputToCmdNode(Node):
    def __init__(self):
        super().__init__('lava_output_to_cmd')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)

        # Set up ZeroMQ subscriber
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.SUB)
        self.zmq_socket.setsockopt(zmq.RCVHWM, 1)  # Set max queue size to 1
        self.zmq_socket.connect("tcp://localhost:5556")  # Connect to the Lava SNN output port
        self.zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

        # Timer to poll ZeroMQ messages periodically
        self.timer = self.create_timer(0.1, self.receive_lava_command)

    def receive_lava_command(self):
        """Poll for commands from the Lava SNN and publish as ROS2 messages."""
        try:
            latest_command = None

            # Drain the queue to get the latest message
            while True:
                if self.zmq_socket.poll(timeout=1):  # Non-blocking check for messages
                    latest_command = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)
                else:
                    break

            if latest_command:  # Process only the latest message
                self.get_logger().info(f"Processing latest command: {latest_command}")
                move_cmd = self.interpret_snn_output(latest_command)
                self.publisher_.publish(move_cmd)

            else:
                self.get_logger().info("No commands received from Lava SNN.")
        except zmq.Again:  # Handle the case where no messages are available
            self.get_logger().info("No messages in the ZeroMQ queue.")
        except zmq.ZMQError as e:
            self.get_logger().error(f"ZeroMQ error: {e}")

    def interpret_snn_output(self, command):
        move_cmd = Twist()
        if command == "left":
            move_cmd.angular.z = 0.5  # Turn left
        elif command == "right":
            move_cmd.angular.z = -0.5  # Turn right
        elif command == "forward":
            move_cmd.linear.x = 0.2  # Move forward
        elif command == "stop":
            move_cmd.linear.x = 0.0  # Stop linear motion
            move_cmd.angular.z = 0.0  # Stop angular motion
        return move_cmd

    def stop(self):
        """Clean up resources."""
        self.zmq_socket.close()
        self.zmq_context.term()

def main(args=None):
    rclpy.init(args=args)
    node = LavaOutputToCmdNode()
    rclpy.spin(node)
    node.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
