import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist  # Add import for Twist messages
import asyncio
import websockets
import json
import threading
import math
from websockets.exceptions import ConnectionClosedError, ConnectionClosedOK

class RoverTeleopNode(Node):
    def __init__(self):
        super().__init__('rover_teleop_node')
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.connected_clients = set()

        self.ws_thread = threading.Thread(target=self.start_websocket_server)
        self.ws_thread.daemon = True
        self.ws_thread.start()

    def start_websocket_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        loop.run_until_complete(self.websocket_server())

    async def websocket_server(self):
        server = await websockets.serve(
            self.handle_client,
            "0.0.0.0",
            8765,
            ping_interval=20,
            ping_timeout=10
        )
        self.get_logger().info("WebSocket server started on ws://0.0.0.0:8765")
        await server.wait_closed()

    async def handle_client(self, websocket):
        self.connected_clients.add(websocket)
        self.get_logger().info(f"Client connected. Total clients: {len(self.connected_clients)}")
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    angle = 0
                    speed = 0
                    
                    if data['input_type'] == 'keyboard':
                        keyboard_data = data['data']

                        angle = keyboard_data.get('angle', 0)
                        speed = keyboard_data.get('speed', 0)

                        if speed > 0:
                            self.get_logger().info(f"Movement: angle: {angle}, speed: {speed}")

                            if angle == 0:
                                self.get_logger().info("Direction: Forward")
                            elif angle == 45:
                                self.get_logger().info("Direction: Forward-Right")
                            elif angle == 90:
                                self.get_logger().info("Direction: Right")
                            elif angle == 135:
                                self.get_logger().info("Direction: Back-Right")
                            elif angle == 180:
                                self.get_logger().info("Direction: Back")
                            elif angle == 225:
                                self.get_logger().info("Direction: Back-Left")
                            elif angle == 270:
                                self.get_logger().info("Direction: Left")
                            elif angle == 315:
                                self.get_logger().info("Direction: Forward-Left")
                                
                    elif data['input_type'] == 'controller':
                        left_y = data['data'].get('left_stick_y', 0)
                        right_x = data['data'].get('right_stick_x', 0)
                        
                        twist = Twist()
                        twist.linear.x = -left_y  # Invert if needed based on joystick orientation
                        twist.angular.z = right_x
                        self.cmd_vel_publisher.publish(twist)
                        
                        if abs(left_y) > 0.1 or abs(right_x) > 0.1:
                            self.get_logger().info(f"Joystick: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
                        continue
                    
                    self.publish_twist_from_angle_speed(angle, speed)
                    
                except json.JSONDecodeError:
                    self.get_logger().error("Invalid JSON received")
                except Exception as e:
                    self.get_logger().error(f"Error handling message: {e}")
                    
        except ConnectionClosedOK:
            self.get_logger().info("Client disconnected normally")
        except ConnectionClosedError as e:
            self.get_logger().error(f"Connection closed unexpectedly: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
        finally:
            self.connected_clients.remove(websocket)
            self.get_logger().info(f"Client disconnected. Total clients: {len(self.connected_clients)}")
            
            self.publish_twist_from_angle_speed(0, 0)

    def publish_twist_from_angle_speed(self, angle, speed):
        """Convert angle (0-360) and speed (0-100) to Twist message"""
        twist = Twist()
        
        if speed <= 0:
            self.cmd_vel_publisher.publish(twist)
            return
        
        norm_speed = speed / 100.0
        
        angle_rad = math.radians(angle)
        
        # Calculate linear and angular components
        # Forward/backward component
        twist.linear.x = norm_speed * math.cos(angle_rad)
        
        # Left/right rotation component
        # Note: Negative for clockwise/right turn, positive for counterclockwise/left turn
        twist.angular.z = -norm_speed * math.sin(angle_rad)
        
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f"Publishing cmd_vel: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = RoverTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'cmd_vel_publisher'):
            node.cmd_vel_publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()