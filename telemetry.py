import logging
import socket
import json
import time
import threading

class Telemetry:
    def __init__(self, telemetry_ip, telemetry_port, log_file="telemetry.log", transmission_interval=1):
        """
        Initializes the telemetry module.
        :param telemetry_ip: IP address for UDP telemetry transmission.
        :param telemetry_port: Port for UDP telemetry transmission.
        :param log_file: Path to the telemetry log file.
        :param transmission_interval: Interval (seconds) between telemetry transmissions.
        """
        self.telemetry_ip = telemetry_ip
        self.telemetry_port = telemetry_port
        self.transmission_interval = transmission_interval

        # Log file path (ensure it's a valid string or path)
        self.log_file = "telemetry.log"

        # Set up logging
        self.logger = logging.getLogger("TelemetryLogger")
        self.logger.setLevel(logging.INFO)
        file_handler = logging.FileHandler(self.log_file)  # Ensure log_file is a string
        file_handler.setFormatter(logging.Formatter('%(message)s'))
        self.logger.addHandler(file_handler)

        # UDP socket for telemetry transmission
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Thread for telemetry transmission
        self.telemetry_data = None
        self.is_running = False
        self.telemetry_thread = threading.Thread(target=self.telemetry_loop, daemon=True)

    def update_telemetry_data(self, data):
        """
        Updates the current telemetry data.
        :param data: Dictionary containing the latest telemetry data.
        """
        self.telemetry_data = data

    def start(self):
        """
        Starts the telemetry transmission loop.
        """
        print("Starting telemetry transmission...")
        self.is_running = True
        self.telemetry_thread.start()

    def stop(self):
        """
        Stops the telemetry transmission loop.
        """
        print("Stopping telemetry transmission...")
        self.is_running = False
        self.telemetry_thread.join()

    def log_telemetry(self):
        """
        Logs the current telemetry data to the log file.
        """
        if self.telemetry_data:
            self.logger.info(json.dumps(self.telemetry_data))

    def send_telemetry(self):
        """
        Sends the current telemetry data over UDP.
        """
        if self.telemetry_data:
            self.udp_socket.sendto(json.dumps(self.telemetry_data).encode("utf-8"), (self.telemetry_ip, self.telemetry_port))

    def telemetry_loop(self):
        """
        Periodically transmits and logs telemetry data.
        """
        while self.is_running:
            if self.telemetry_data:
                self.log_telemetry()
                self.send_telemetry()
            time.sleep(self.transmission_interval)


if __name__ == "__main__":
    # Configuration
    TELEMETRY_IP = "127.0.0.1"  # Replace with the actual IP address
    TELEMETRY_PORT = 50055  # Replace with the actual port

    # Initialize the telemetry system
    telemetry = Telemetry(TELEMETRY_IP, TELEMETRY_PORT)

    try:
        # Start telemetry
        telemetry.start()

        # Keep running until interrupted
        print("Press Ctrl+C to stop the telemetry system.")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("Shutting down telemetry...")
        telemetry.stop()

    except Exception as e:
        print(f"Unexpected error: {e}")
        telemetry.stop()