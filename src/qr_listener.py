import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class QRCodeSubscriber(Node):
    def __init__(self):
        super().__init__('qr_code_subscriber')
        self.subscription = self.create_subscription(
            String, '/qr_code/data', self.qr_callback, 10)

    def qr_callback(self, msg):
        self.get_logger().info(f'Received QR Code: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#entry_points={
#    'console_scripts': [
#        'qr_listener = rover_bringup.qr_listener:main',
#    ],
#},