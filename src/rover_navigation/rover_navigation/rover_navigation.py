import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math

class RoverNavigationNode(Node):
    def __init__(self):
        super().__init__('rover_navigation_node')
        
        self.subscription = self.create_subscription(
            String,
            'rover_control',
            self.control_callback,
            10
        )
        
        self.get_logger().info('Rover Navigation Node initialized')
        
    def control_callback(self, msg):
        try:
            control_data = json.loads(msg.data)
            
            angle = control_data.get('angle', 0)
            speed = control_data.get('speed', 0)
            
            if speed > 0:
                self.get_logger().info(f'Moving rover at angle: {angle}, speed: {speed}')
                self.move_precise(angle, speed)
            else:
                self.stop()
                
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON received')
        except Exception as e:
            self.get_logger().error(f'Error processing control message: {e}')
    
    def move_precise(self, angle, speed):
        angle_rad = math.radians(angle)
        forward_component = math.cos(angle_rad)
        right_component = math.sin(angle_rad)

        left_motor_speed = speed * (forward_component - right_component)
        right_motor_speed = speed * (forward_component + right_component)

        max_speed = max(abs(left_motor_speed), abs(right_motor_speed))
        if (max_speed > 100):
            left_motor_speed = left_motor_speed / max_speed * 100
            right_motor_speed = right_motor_speed / max_speed * 100

        self.get_logger().info(f'Moving motors at speeds: L={left_motor_speed:.1f}, R={right_motor_speed:.1f}')
        # TODO: Motor control

    def stop(self):
        self.get_logger().info('Stopping rover')
        # TODO: Motor control

def main(args=None):
    rclpy.init(args=args)
    node = RoverNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()