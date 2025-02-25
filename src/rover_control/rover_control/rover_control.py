import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import websockets
import json
import threading
from websockets.exceptions import ConnectionClosedError, ConnectionClosedOK

class RoverControlNode(Node):
    def __init__(self):
        super().__init__('rover_control_node')
        self.control_publisher = self.create_publisher(String, 'rover_control', 10)
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
                    # self.get_logger().info(f"Raw JSON data:\n{json.dumps(data, indent=2)}")                    

                    msg = String()
                    msg.data = message
                    
                    if data['input_type'] == 'keyboard':
                        keyboard_data = data['data']

                        angle = keyboard_data.get('angle', 0)
                        speed = keyboard_data.get('speed', 0)
                        # key_w = keyboard_data.get('key_w', False)
                        # key_a = keyboard_data.get('key_a', False)
                        # key_s = keyboard_data.get('key_s', False)
                        # key_d = keyboard_data.get('key_d', False)

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
                        if abs(left_y) > 0.1:
                            self.get_logger().info(f"Moving at speed: {left_y}")
                    
                    simplified_data = {
                        "angle": angle,
                        "speed": speed
                    }
                    msg = String()
                    msg.data = json.dumps(simplified_data)
                    self.control_publisher.publish(msg)
                    
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

def main(args=None):
    rclpy.init(args=args)
    node = RoverControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()