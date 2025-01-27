import threading
import time
import logging
import socket
import json
import os
import psutil

class Telemetry:
    def __init__(self, ip, port, get_data_callback, log_file="telemetry.log"):
        """
        Initialize the Telemetry module.

        :param ip: The IP address to send telemetry data to.
        :param port: The port to send telemetry data to.
        :param get_data_callback: A callback function to fetch telemetry data.
        :param log_file: Path to the log file for telemetry data.
        """
        self.ip = ip
        self.port = port
        self.get_data_callback = get_data_callback
        self.log_file = log_file
        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.running = False
        self.thread = None

        # Setup logging
        self.logger = logging.getLogger("TelemetryLogger")
        self.logger.setLevel(logging.INFO)
        file_handler = logging.FileHandler(self.log_file)
        file_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
        self.logger.addHandler(file_handler)

    def start(self):
        """Start the telemetry loop in a separate thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.telemetry_loop, daemon=True)
            self.thread.start()
            print("Telemetry loop started.")

    def stop(self):
        """Stop the telemetry loop."""
        self.running = False
        if self.thread:
            self.thread.join()
            print("Telemetry loop stopped.")

    def telemetry_loop(self):
        """Continuously fetch and transmit telemetry data."""
        print("Starting telemetry loop...")
        while self.running:
            try:
                telemetry_data = self.get_data_callback()
                telemetry_data.update(self.get_system_state())

                # Log telemetry data
                self.logger.info(json.dumps(telemetry_data))

                # Send telemetry data
                self.telemetry_socket.sendto(json.dumps(telemetry_data).encode("utf-8"), (self.ip, self.port))
                print(f"Telemetry sent: {telemetry_data}")

                time.sleep(1)  # Send updates every second
            except Exception as e:
                print(f"Error in telemetry loop: {e}")

    def get_system_state(self):
        """Gather system metrics."""
        return {
            "cpu_usage": psutil.cpu_percent(interval=1),
            "memory_available": psutil.virtual_memory().available / (1024 * 1024),  # Available memory in MB
            "memory_total": psutil.virtual_memory().total / (1024 * 1024),  # Total memory in MB
            "disk_usage": psutil.disk_usage('/').percent,  # Disk usage percentage
            "temperature": self.get_temperature(),  # CPU temperature in Celsius
            "uptime": self.get_uptime(),  # System uptime in seconds
        }

    def get_temperature(self):
        """Get the CPU temperature from the Raspberry Pi."""
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                return int(f.read().strip()) / 1000.0  # Convert millidegrees to degrees
        except FileNotFoundError:
            return "N/A"  # Temperature sensor not available

    def get_uptime(self):
        """Calculate system uptime."""
        try:
            with open("/proc/uptime", "r") as f:
                uptime_seconds = float(f.readline().split()[0])
                return uptime_seconds
        except Exception as e:
            return "N/A"

# For standalone testing
if __name__ == "__main__":
    def mock_telemetry_data():
        return {
            "position": (0, 0),
            "heading": "N",
            "battery_level": 100,
            "ultrasound_distance": 50,
            "state": "Idle",
            "proximity_alert": None
        }

    telemetry = Telemetry(ip="127.0.0.1", port=50055, get_data_callback=mock_telemetry_data)
    telemetry.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        telemetry.stop()
