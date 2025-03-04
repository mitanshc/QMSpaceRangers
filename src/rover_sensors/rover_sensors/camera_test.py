from http.server import BaseHTTPRequestHandler, HTTPServer
import cv2
import numpy as np
import time

# Note: This file is not part of the ROS codebase. It is a simple HTTP server that streams a moving rectangle
# as a MJPEG stream in order to test the camera feed in the browser.
class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            frame_count = 0
            try:
                while True:
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    
                    x = (frame_count * 5) % 540
                    cv2.rectangle(frame, (x, 100), (x+100, 300), (0, 0, 255), -1)
                    
                    cv2.putText(frame, f"Test Frame: {frame_count}", (50, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    
                    ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    
                    if ret:
                        self.wfile.write(bytes("--FRAME\r\n", "utf-8"))
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(jpeg)))
                        self.end_headers()
                        self.wfile.write(jpeg.tobytes())
                        self.wfile.write(bytes("\r\n", "utf-8"))
                    
                    frame_count += 1
                    time.sleep(0.05)
            except Exception as e:
                print(f"Connection closed: {str(e)}")
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        pass

def main():
    server = HTTPServer(('0.0.0.0', 8080), MJPEGHandler)
    print("Mock MJPEG server started on port 8080")
    print("Access at http://localhost:8080/stream")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Server stopped")

if __name__ == "__main__":
    main()