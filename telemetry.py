import threading
import logging
import json
import time
import os
import psutil
import socket
import rover.rover

import logging
# Use existing logger
logger = logging.getLogger(__name__)

class Telemetry:
    def __init__(self, telemetry_ip, telemetry_port, get_telemetry_data, send_rate=1.0):
        """
        Initializes the Telemetry object.

        Args:
            telemetry_ip (str): IP address for telemetry data transmission.
            telemetry_port (int): Port for telemetry data transmission.
            get_telemetry_data (callable): Function to fetch telemetry data.
            send_rate (float): Rate in seconds at which telemetry data is sent.
        """
        self.telemetry_ip = telemetry_ip
        self.telemetry_port = telemetry_port
        self.get_telemetry_data = get_telemetry_data
        self.send_rate = send_rate

        # Create a separate telemetry logger
        self.logger = logging.getLogger("TelemetryLogger")
        self.logger.setLevel(logging.INFO)  # Log only telemetry info
        file_handler = logging.FileHandler("telemetry.log", mode="w")
        file_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
        # Prevent log messages from propagating to the root logger
        self.logger.propagate = False  
        # Clear previous handlers (prevents duplicate logs if re-initialized)
        if not self.logger.handlers:
            self.logger.addHandler(file_handler)

        self.telemetry_socket = None
        self.running = False
        self.thread = None

    def start(self):
        """Starts the telemetry transmission in a separate thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.telemetry_loop, daemon=True)
            self.thread.start()
            logger.info("Telemetry transmission started.")

    def stop(self):
        """Stops the telemetry transmission."""
        self.running = False
        if self.thread:
            self.thread.join()
            logger.info("Telemetry transmission stopped.")

    def telemetry_loop(self):
        """Telemetry loop that sends data at the specified send rate."""
        logger.info("Starting telemetry loop...")
        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        i=0
        while self.running:
            telemetry_data = self.get_telemetry_data()
            telemetry_data.update(self.get_system_state())
            self.logger.info(json.dumps(telemetry_data))
            self.telemetry_socket.sendto(json.dumps(telemetry_data).encode("utf-8"), (self.telemetry_ip, self.telemetry_port))
            if i%20==0:
                logger.info(f"Telemetry sent: {telemetry_data}")
            i+=1
            time.sleep(self.send_rate)

    def get_system_state(self):
        """Gathers system state metrics."""
        return {
            "cpu_usage": psutil.cpu_percent(interval=None),
            "memory_available": psutil.virtual_memory().available / (1024 * 1024),  # MB
            "memory_total": psutil.virtual_memory().total / (1024 * 1024),  # MB
            "disk_usage": psutil.disk_usage('/').percent,
            "temperature": self.get_temperature(),
            "uptime": self.get_uptime(),
        }

    @staticmethod
    def get_temperature():
        """Fetches the CPU temperature."""
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                return int(f.read().strip()) / 1000.0  # Convert millidegrees to degrees
        except FileNotFoundError:
            return "N/A"

    @staticmethod
    def get_uptime():
        """Fetches the system uptime."""
        try:
            with open("/proc/uptime", "r") as f:
                uptime_seconds = float(f.readline().split()[0])
                return uptime_seconds
        except Exception:
            return "N/A"

if __name__ == "__main__":

    def mock_telemetry_data():
        return {
            "position": (0, 0),
            "heading": "N",
            "battery_level": 100,
            "ultrasound_distance": rover.getDistance(),
            "state": "Idle",
            "proximity_alert": None
        }

    TELEMETRY_IP = "127.0.0.1"
    TELEMETRY_PORT = 50055
    SEND_RATE = 2.0  # Send telemetry every 2 seconds

    telemetry = Telemetry(TELEMETRY_IP, TELEMETRY_PORT, mock_telemetry_data, send_rate=SEND_RATE)
    telemetry.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        telemetry.stop()
