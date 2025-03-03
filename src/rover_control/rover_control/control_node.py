import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import RPi.GPIO as GPIO

class RoverControlNode(Node):
    def __init__(self):
        super().__init__('rover_control')

        self.left_motors = [
            {'pwm': 12, 'dir1': 16, 'dir2': 18},  # Left motor 1
            {'pwm': 32, 'dir1': 33, 'dir2': 35},  # Left motor 2
            {'pwm': 36, 'dir1': 37, 'dir2': 38}   # Left motor 3
        ]

        self.right_motors = [
            {'pwm': 13, 'dir1': 22, 'dir2': 24},  # Right motor 1
            {'pwm': 40, 'dir1': 41, 'dir2': 42},  # Right motor 2
            {'pwm': 43, 'dir1': 44, 'dir2': 45}   # Right motor 3
        ]

        self.track_width = 0.5  # need to change
        self.max_linear_speed = 1.0  # m/s
        self.max_angular_speed = 1.5 # rad/s

        self.setup_gpio()

        # Subscribe to /cmd_vel (navigation & teleop output)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Subscribe to /sensor_data (sensor node output)
        self.sensor_sub = self.create_subscription(
            String, '/sensor_data', self.sensor_callback, 10)

        # Publisher for motor commands
        self.motor_pub = self.create_publisher(String, '/motor_commands', 10)

        self.get_logger().info("Rover Control Node has started!")
    
    def setup_gpio(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        self.left_pwms = []
        self.right_pwms = []

        for motor in self.left_motors:
            for pin in motor.values():
                GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(motor['pwm'], 100)  # 100Hz frequency
            pwm.start(0)  # Start with 0% duty cycle
            self.left_pwms.append(pwm)
            
        for motor in self.right_motors:
            for pin in motor.values():
                GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(motor['pwm'], 100)
            pwm.start(0)
            self.right_pwms.append(pwm)

        self.get_logger().info("GPIO setup complete")

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f"Received velocity command: linear={msg.linear.x}, angular={msg.angular.z}")
        
        linear_x = max(min(msg.linear.x, self.max_linear_speed), -self.max_linear_speed)
        angular_z = max(min(msg.angular.z, self.max_angular_speed), -self.max_angular_speed)
        
        left_velocity = linear_x - (angular_z * self.track_width / 2.0)
        right_velocity = linear_x + (angular_z * self.track_width / 2.0)
        
        left_speed = (left_velocity / self.max_linear_speed) * 100.0
        right_speed = (right_velocity / self.max_linear_speed) * 100.0
        
        self.set_motor_speeds(left_speed, right_speed)

        motor_command = f"Move: linear={msg.linear.x}, angular={msg.angular.z}"
        self.motor_pub.publish(String(data=motor_command))

    def sensor_callback(self, msg):
        self.get_logger().info(f"Received sensor data: {msg.data}")

    def set_motor_speeds(self, left_speed, right_speed):
        for i, motor in enumerate(self.left_motors):
            self.set_single_motor(left_speed, motor['dir1'], motor['dir2'], self.left_pwms[i])
        
        for i, motor in enumerate(self.right_motors):
            self.set_single_motor(right_speed, motor['dir1'], motor['dir2'], self.right_pwms[i])
            
        self.get_logger().debug(f"Setting motors: left={left_speed:.1f}, right={right_speed:.1f}")

    def set_single_motor(self, speed, dir1_pin, dir2_pin, pwm):
        if speed > 0:
            GPIO.output(dir1_pin, GPIO.HIGH)
            GPIO.output(dir2_pin, GPIO.LOW)
        else:
            GPIO.output(dir1_pin, GPIO.LOW)
            GPIO.output(dir2_pin, GPIO.HIGH)
        
        pwm_value = min(abs(speed), 100)  # Ensure speed doesn't exceed 100%
        pwm.ChangeDutyCycle(pwm_value)

    def stop_motors(self):
        self.get_logger().info('Stopping all motors')
        for i, motor in enumerate(self.left_motors):
            self.left_pwms[i].ChangeDutyCycle(0)
        
        for i, motor in enumerate(self.right_motors):
            self.right_pwms[i].ChangeDutyCycle(0)
            
        self.motor_pub.publish(String(data="Motors STOPPED"))

    def cleanup(self):
        self.stop_motors()
        for pwm in self.left_pwms + self.right_pwms:
            pwm.stop()
        GPIO.cleanup()
        self.get_logger().info("Motor GPIO resources cleaned up")

def main(args=None):
    rclpy.init(args=args)
    node = RoverControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
