import io
import time
import threading
import http.server
from http import HTTPStatus
from picamera2 import Picamera2
from PIL import Image

import logging
# Use existing logger
logger = logging.getLogger(__name__)

class MJPEGStreamHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream':
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.end_headers()

            try:
                while not self.server.should_stop:
                    frame = self.server.get_frame()
                    if frame is None:
                        time.sleep(0.1)
                        continue

                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")

                    time.sleep(0.5)  # Adjust frame rate
            except Exception as e:
                logger.info(f"Client disconnected: {e}")

class MJPEGStreamServer(http.server.HTTPServer):
    def __init__(self, server_address, handler_class):
        super().__init__(server_address, handler_class)
        self.frame = None
        self.lock = threading.Lock()
        self.should_stop = False

        self.running = False
        self.thread = None

    def update_frame(self, frame):
        """Update the latest frame in memory."""
        with self.lock:
            self.frame = frame

    def get_frame(self):
        """Retrieve the latest frame safely."""
        with self.lock:
            return self.frame

    def start(self):
        """Starts the MJPEG stream server in a separate thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.serve_forever, daemon=True)
            self.thread.start()
            logger.info("MJPEG Stream Server started.")

    def stop(self):
        """Stops the MJPEG stream server and thread."""
        self.should_stop = True
        self.shutdown()
        if self.thread:
            self.thread.join()
        logger.info("MJPEG Stream Server stopped.")

def start_camera_stream(server):
    """Continuously captures frames and updates the MJPEG stream & local memory."""
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
    picam2.start()

    try:
        while not server.should_stop:
            image_array = picam2.capture_array()
            image = Image.fromarray(image_array).rotate(270, expand=True)

            # Store in buffer
            buffer = io.BytesIO()
            image.save(buffer, format="JPEG")
            frame = buffer.getvalue()

            # Update MJPEG stream + local access frame
            server.update_frame(frame)

            time.sleep(0.5)  # Adjust frame rate
    except KeyboardInterrupt:
        logger.info("Stopping camera stream...")
    finally:
        picam2.stop()

def start_mjpeg_server(server):
    """Run the MJPEG server"""
    try:
        server.serve_forever()
    except Exception as e:
        logger.info(f"Error in MJPEG server: {e}")

def start_server():
    """Starts the MJPEG stream and camera capture"""
    server_address = ('', 8080)  # Bind to all interfaces on port 8080
    server = MJPEGStreamServer(server_address, MJPEGStreamHandler)

    # Start the camera stream in a separate thread
    camera_thread = threading.Thread(target=start_camera_stream, args=(server,), daemon=True)
    camera_thread.start()

    logger.info("Starting MJPEG stream on http://localhost:8080/stream")
    try:
        start_mjpeg_server(server)
    except KeyboardInterrupt:
        logger.info("Shutting down server...")
        server.stop()

if __name__ == "__main__":
    start_server()