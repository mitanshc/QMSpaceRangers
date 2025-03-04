#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
import time

class MJPEGHandler(BaseHTTPRequestHandler):

    current_frame = None

    @staticmethod
    def set_current_frame(frame):
        MJPEGHandler.current_frame = frame

    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            try:
                while True:
                    if MJPEGHandler.current_frame is not None:
                        ret, jpeg = cv2.imencode('.jpg', MJPEGHandler.current_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        if not ret:
                            continue

                        self.wfile.write(bytes("--FRAME\r\n", "utf-8"))
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(jpeg)))
                        self.end_headers()
                        self.wfile.write(jpeg.tobytes())
                        self.wfile.write(bytes("\r\n", "utf-8"))
                        time.sleep(0.05) # 20fps
            except Exception as e:
                print(f"Connection closed: {str(e)}")
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        pass
    
class LiveFeedPublisher(Node):
    def __init__(self): 
        super().__init__("live_feed_publisher")
        self.publisher = self.create_publisher(Image, 'live_feed',10)
        self.bridge = CvBridge() 

        #self.cap =cv2.VideoCapture("libcamerassrc ! video/x-raw,width=320,height=240,framerates=5/1 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)  #to be changed for raspberry pi
        #self.cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw,width=320,height=240,framerate=5/1 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
        self.cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)


        if not self.cap.isOpened():
            self.get_logger().error("Unable to open camera") 
        else:
            self.get_logger().info("Camera opened")

        
        self.qr_detector =cv2.QRCodeDetector()
        self.qr_publisher = self.create_publisher(String, 'qr_codes',10)

        self.http_port = 8080
        self.start_http_server()

        timer_period = 0.1 #timer for ROS2
        self.timer =self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"Camera stream available at: http://<rover-ip>:{self.http_port}/stream")

    def start_http_server(self):
        self.http_server = HTTPServer(('0.0.0.0', self.http_port), MJPEGHandler)
        self.http_thread = threading.Thread(target=self.http_server.serve_forever)
        self.http_thread.daemon = True
        self.http_thread.start()

    def timer_callback(self): #keeps camera open until closed
        ret, frame = self.cap.read()
        if ret: #True it suscefully connected
            try:
                MJPEGHandler.set_current_frame(frame)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 

                QR_info,bbox,_ = self.qr_detector.detectAndDecode(frame)#detect and decoder return three values

                if bbox is not None:
                    bbox =bbox.astype(int)
                    cv2.polylines(frame, [bbox], isClosed=True, color=(0,255,0,), thickness=(3))

                    if QR_info:
                        cv2.putText(frame, QR_info, (bbox[0][0][0], bbox[0][0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),2)
                        self.get_logger().info("code detected: {Qr_info}")

                        msg =String()
                        msg.data = QR_info 
                        self.qr_publisher.publish(msg)

                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8") 

                self.publisher.publish(ros_image) 
                #self.get_logger().info("Frames published")
                

                cv2.imshow("live_feed", frame) 
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f"Cv Bridge Error:{str(e)}")
        else: #if False then failed to connect
            self.get_logger().info("No frames received from webcam")

    def cleanup(self):
        if hasattr(self, 'http_server'):
            self.http_server.shutdown()
            self.http_server.server_close()

        if hasattr(self, 'cap'):
            self.cap.release()

        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = LiveFeedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
if __name__== "__main__":
    main()
 
