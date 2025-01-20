import threading
import time
import socket
import json
import platform

if platform.system() == "Linux":
    from controllers import MotorController, SensorController, NavigationController
else:
    from dummy import MotorController, SensorController, NavigationController

# State Machine States
class RoverState:
    IDLE = "Idle"
    EXPLORING = "Exploring"
    AVOIDING_OBSTACLE = "AvoidingObstacle"
    PURSUING_RESOURCE = "PursuingResource"

# AutoPi Class for Autonomous Control
class AutoPi:
    def __init__(self, telemetry_ip, telemetry_port):
        self.state = RoverState.IDLE
        self.motor_controller = MotorController()
        self.sensor_controller = SensorController()
        self.navigation_controller = NavigationController()
        self.current_path = []  # Exploration path
        self.target_resource = None
        self.lock = threading.Lock()

        # Telemetry
        self.telemetry_ip = telemetry_ip
        self.telemetry_port = telemetry_port
        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.telemetry_thread = threading.Thread(target=self.telemetry_loop, daemon=True)
        self.telemetry_thread.start()

    def set_state(self, new_state):
        with self.lock:
            print(f"State change: {self.state} -> {new_state}")
            self.state = new_state

    def telemetry_loop(self):
        while True:
            telemetry_data = {
                "position": self.sensor_controller.get_position(),
                "heading": self.sensor_controller.get_heading(),
                "battery_level": self.sensor_controller.get_battery_level(),
                "ultrasound_distance": self.sensor_controller.get_ultrasound_distance(),
            }
            self.telemetry_socket.sendto(json.dumps(telemetry_data).encode("utf-8"), (self.telemetry_ip, self.telemetry_port))
            time.sleep(1)  # Send updates every second

    def exploration_mode(self):
        while self.state == RoverState.EXPLORING:
            obstacle_detected = self.sensor_controller.check_for_obstacles()
            if obstacle_detected:
                self.set_state(RoverState.AVOIDING_OBSTACLE)
                return

            self.navigation_controller.follow_path(self.current_path)
            resource = self.sensor_controller.detect_resource()
            if resource:
                self.target_resource = resource
                self.set_state(RoverState.PURSUING_RESOURCE)
                return

            time.sleep(0.1)

    def avoidance_mode(self):
        while self.state == RoverState.AVOIDING_OBSTACLE:
            if self.navigation_controller.navigate_around_obstacle():
                self.set_state(RoverState.EXPLORING)
                return
            time.sleep(0.1)

    def pursuit_mode(self):
        while self.state == RoverState.PURSUING_RESOURCE:
            if self.navigation_controller.move_to(self.target_resource):
                print(f"Reached resource: {self.target_resource}")
                self.target_resource = None
                self.set_state(RoverState.EXPLORING)
                return
            time.sleep(0.1)

    def run(self):
        while True:
            if self.state == RoverState.EXPLORING:
                self.exploration_mode()
            elif self.state == RoverState.AVOIDING_OBSTACLE:
                self.avoidance_mode()
            elif self.state == RoverState.PURSUING_RESOURCE:
                self.pursuit_mode()
            else:
                time.sleep(0.1)

if __name__ == "__main__":
    TELEMETRY_IP = "127.0.0.1"  # Replace with actual IP address
    TELEMETRY_PORT = 50055  # Replace with actual port

    pi = AutoPi(TELEMETRY_IP, TELEMETRY_PORT)

    # Start in exploring mode
    pi.set_state(RoverState.EXPLORING)

    # Run the rover in a separate thread
    rover_thread = threading.Thread(target=pi.run)
    rover_thread.start()

    # Mocking external commands (e.g., stop rover)
    while True:
        command = input("Enter command: ")
        if command.lower() == "stop":
            pi.set_state(RoverState.IDLE)
            break

    rover_thread.join()