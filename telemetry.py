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
        self.log_file = log_file
        self.transmission_interval = transmission_interval

        # Logging configuration
        self.logger = logging.getLogger("TelemetryLogger")
        self.logger.setLevel(logging.INFO)
        file_handler = logging.FileHandler(self.log_file)
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