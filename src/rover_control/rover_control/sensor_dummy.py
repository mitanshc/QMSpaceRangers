import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DummySensorNode(Node):
    def __init__(self):
        super().__init__('dummy_sensor')
        self.publisher_ = self.create_publisher(String, '/sensor_data', 10)
        self.timer = self.create_timer(2.0, self.publish_fake_data)

    def publish_fake_data(self):
        msg = String()
        msg.data = "Fake Sensor Data: Obstacle detected!"
        self.publisher_.publish(msg)
        self.get_logger().info("Published fake sensor data")

def main(args=None):
    rclpy.init(args=args)
    node = DummySensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
