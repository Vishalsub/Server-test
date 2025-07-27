import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading

cap = cv2.VideoCapture(0)

class MJPEGStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != '/':
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header(
            'Content-type',
            'multipart/x-mixed-replace; boundary=frame'
        )
        self.end_headers()

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Encode frame as JPEG
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue

            self.wfile.write(b'--frame\r\n')
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg.tobytes())
            self.wfile.write(b'\r\n')

def start_server():
    server = HTTPServer(('0.0.0.0', 8000), MJPEGStreamHandler)
    print("🚀 MJPEG Stream available at: http://<your-ip>:8000")
    server.serve_forever()

if __name__ == '__main__':
    thread = threading.Thread(target=start_server)
    thread.daemon = True
    thread.start()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("🛑 Shutting down")
        cap.release()