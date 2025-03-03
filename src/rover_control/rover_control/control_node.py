import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class RoverControlNode(Node):
    def __init__(self):
        super().__init__('rover_control')

        # Subscribe to /cmd_vel (navigation & teleop output)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Subscribe to /sensor_data (sensor node output)
        self.sensor_sub = self.create_subscription(
            String, '/sensor_data', self.sensor_callback, 10)

        # Publisher for motor commands
        self.motor_pub = self.create_publisher(String, '/motor_commands', 10)

        self.get_logger().info("Rover Control Node has started!")

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f"Received velocity command: linear={msg.linear.x}, angular={msg.angular.z}")
        # Convert Twist message to a motor command
        motor_command = f"Move: linear={msg.linear.x}, angular={msg.angular.z}"
        self.motor_pub.publish(String(data=motor_command))

    def sensor_callback(self, msg):
        self.get_logger().info(f"Received sensor data: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = RoverControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
