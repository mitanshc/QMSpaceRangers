#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

class LiveFeedPublisher(Node):
    def __init__(self): 
        super().__init__("live_feed_publisher")
        self.publisher = self.create_publisher(Image, 'live_feed',10)
        self.bridge = CvBridge() 

        self.cap =cv2.VideoCapture("libcamerassrc ! video/x-raw,width=320,height=240,framerates=5/1 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)  #to be changed for raspberry pi
        if not self.cap.isOpened():
            self.get_logger().error("Unable to open camera") 
        else:
            self.get_logger().info("Camera opened")

        
        self.qr_detector =cv2.QRCodeDetector()
        self.qr_publisher = self.create_publisher(String, 'qr_codes',10)

        timer_period = 0.1 #timer for ROS2
        self.timer =self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self): #keeps camera open until closed
        ret, frame = self.cap.read()
        if ret: #True it suscefully connected
            try:
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


def main(args=None):
    rclpy.init(args=args)
    node = LiveFeedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__== "__main__":
    main()
 
