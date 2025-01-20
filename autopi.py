import threading
import time
import socket
import json
import platform
import argparse

from planning import AStarPlanner
from obstacle import ObstacleDetector
from path import generate_expanding_square_path, generate_random_walk_path, generate_sine_wave_path, generate_spiral_pattern, generate_zigzag_pattern, generate_straight_line_path

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
    SIMULATING = "Simulating"

# AutoPi Class for Autonomous Control
class AutoPi:
    def __init__(self, telemetry_ip, telemetry_port, debug_mode=False, path_type="straight_line", sim_mode=False):
        print("Initializing AutoPi...")
        self.state = RoverState.IDLE
        self.motor_controller = MotorController()
        self.sensor_controller = SensorController()
        self.navigation_controller = NavigationController()
        self.current_path = []  # Exploration path
        self.target_resource = None
        self.lock = threading.Lock()
        self.map_center = (0, 0)  # Rover's position in the local map
        self.grid_size = 20  # Define grid size for local map
        self.obstacles = set()
        self.planner = AStarPlanner(self.grid_size)
        self.debug_mode = debug_mode
        self.sim_mode = sim_mode
        self.heading = "N"  # Default heading is North

        # Path selection
        self.path_type = path_type

        # Obstacle Detector
        self.obstacle_detector = ObstacleDetector(self.sensor_controller.get_ultrasound_distance,
                                                  self.sensor_controller.detect_resource)
        self.obstacle_detector.start()

        # Telemetry
        self.telemetry_ip = telemetry_ip
        self.telemetry_port = telemetry_port
        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.telemetry_thread = threading.Thread(target=self.telemetry_loop, daemon=True)
        self.telemetry_thread.start()
        print("AutoPi initialized.")

        # Draw initial map if in debug mode
        if self.debug_mode:
            self.display_debug_info()

    def set_state(self, new_state):
        with self.lock:
            print(f"State change: {self.state} -> {new_state}")
            self.state = new_state

    def telemetry_loop(self):
        print("Starting telemetry loop...")
        while True:
            proximity_alert = self.obstacle_detector.get_alert_source() if self.obstacle_detector.is_alerted() else None
            telemetry_data = {
                "position": self.map_center,
                "heading": self.heading,
                "battery_level": self.sensor_controller.get_battery_level(),
                "ultrasound_distance": self.sensor_controller.get_ultrasound_distance(),
                "state": self.state,  # Add current state to telemetry
                "proximity_alert": proximity_alert  # Add proximity indicator
            }
            print(f"Telemetry data: {telemetry_data}")
            self.telemetry_socket.sendto(json.dumps(telemetry_data).encode("utf-8"), (self.telemetry_ip, self.telemetry_port))
            time.sleep(1)  # Send updates every second

    def update_map(self):
        print("Updating map...")
        if self.current_path:
            next_position = self.current_path.pop(0)
            shift_x = next_position[0] - self.map_center[0]
            shift_y = next_position[1] - self.map_center[1]
            self.obstacles = {(x - shift_x, y - shift_y) for (x, y) in self.obstacles}
            self.map_center = next_position
            print(f"Environment shifted to keep rover centered at {self.map_center}")

    def display_debug_info(self):
        if self.debug_mode:
            print("Local Map:")
            map_offset = self.grid_size // 2
            for y in range(-map_offset, map_offset):
                row = ""
                for x in range(-map_offset, map_offset):
                    map_pos = (self.map_center[0] + x, self.map_center[1] + y)
                    if map_pos in self.obstacles:
                        row += "X "
                    elif map_pos == self.map_center:
                        row += "R "
                    elif map_pos in self.current_path:
                        row += "* "
                    else:
                        row += ". "
                print(row)
            print(f"Current Path: {self.current_path}")

    def generate_path(self):
        """
        Generate a path based on the selected path type.
        """
        if self.path_type == "random_walk":
            return generate_random_walk_path(steps=50, step_size=1, start_position=self.map_center)
        elif self.path_type == "spiral":
            return generate_spiral_pattern(step_size=1, num_turns=10, start_position=self.map_center)
        elif self.path_type == "zigzag":
            return generate_zigzag_pattern(step_size=1, width=5, height=3, start_position=self.map_center)
        elif self.path_type == "straight_line":
            return generate_straight_line_path(length=10, step_size=1, start_position=self.map_center)
        elif self.path_type == "sine_wave":
            return generate_sine_wave_path(amplitude=5, wavelength=10, total_distance=50, step_size=1, start_position=self.map_center)
        elif self.path_type == "expanding_square":
            return generate_expanding_square_path(step_size=1, num_layers=3, start_position=self.map_center)
        else:
            raise ValueError(f"Unknown path type: {self.path_type}")

    def exploration_mode(self):
        print("Entering exploration mode...")
        while self.state == RoverState.EXPLORING:
            if self.obstacle_detector.is_alerted():
                print(f"Obstacle detected! Source: {self.obstacle_detector.get_alert_source()}")
                self.obstacle_detector.reset_alert()
                self.set_state(RoverState.AVOIDING_OBSTACLE)
                return

            self.current_path = self.generate_path()
            print(f"Generated path: {self.current_path}")

            while self.current_path and self.state == RoverState.EXPLORING:
                self.update_map()
                self.display_debug_info()
                resource = self.sensor_controller.detect_resource()
                if resource:
                    print(f"Resource detected: {resource}")
                    self.target_resource = resource
                    self.set_state(RoverState.PURSUING_RESOURCE)
                    return
                time.sleep(0.5)

    def simulation_mode(self):
        print("Entering simulation mode...")
        while self.state == RoverState.SIMULATING:
            if not self.current_path:
                self.current_path = self.generate_path()
                print(f"Generated path for simulation: {self.current_path}")

            if self.current_path:
                next_waypoint = self.current_path.pop(0)
                print(f"Simulating move to {next_waypoint}")
                self.map_center = next_waypoint
                self.display_debug_info()
                time.sleep(1)  # Simulate movement delay

    def avoidance_mode(self):
        print("Entering avoidance mode...")
        # Replan logic to avoid obstacle and rejoin original path
        while self.state == RoverState.AVOIDING_OBSTACLE:
            avoidance_goal = (self.map_center[0] + 2, self.map_center[1] + 2)  # Example: move diagonally to avoid
            avoidance_path = self.planner.plan(self.map_center, avoidance_goal, self.obstacles)
            print(f"Avoidance path: {avoidance_path}")

            while avoidance_path:
                next_position = avoidance_path.pop(0)
                self.current_path.insert(0, next_position)  # Reinsert planned path ahead
                self.update_map()
                self.display_debug_info()

                if not self.obstacle_detector.is_alerted():
                    print("Obstacle cleared. Returning to planned path.")
                    self.set_state(RoverState.EXPLORING)
                    return

            time.sleep(0.5)

    def pursuit_mode(self):
        print("Entering pursuit mode...")
        while self.state == RoverState.PURSUING_RESOURCE:
            if self.navigation_controller.move_to(self.target_resource):
                print(f"Reached resource: {self.target_resource}")
                self.target_resource = None
                self.set_state(RoverState.EXPLORING)
                return
            time.sleep(0.1)

    def run(self):
        print("Starting main control loop...")
        while True:
            if self.state == RoverState.EXPLORING:
                self.exploration_mode()
            elif self.state == RoverState.AVOIDING_OBSTACLE:
                self.avoidance_mode()
            elif self.state == RoverState.PURSUING_RESOURCE:
                self.pursuit_mode()
            elif self.state == RoverState.SIMULATING:
                self.simulation_mode()
            else:
                print("Rover is idle.")
                time.sleep(0.1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AutoPi Rover")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode to display mapping grid and path planning.")
    parser.add_argument("--path", type=str, default="straight_line", help="Select path type: random_walk, spiral, zigzag, straight_line, sine_wave, expanding_square")
    parser.add_argument("--sim", action="store_true", help="Enable simulation mode.")
    args = parser.parse_args()

    TELEMETRY_IP = "127.0.0.1"  # Replace with actual IP address
    TELEMETRY_PORT = 50055  # Replace with actual port

    print("Initializing rover...")
    pi = AutoPi(TELEMETRY_IP, TELEMETRY_PORT, debug_mode=args.debug, path_type=args.path, sim_mode=args.sim)

    # Start in appropriate mode
    initial_state = RoverState.SIMULATING if args.sim else RoverState.EXPLORING
    print(f"Setting initial state to {initial_state}...")
    pi.set_state(initial_state)

    # Run the rover in a separate thread
    rover_thread = threading.Thread(target=pi.run)
    rover_thread.start()

    # Mocking external commands (e.g., stop rover)
    while True:
        command = input("Enter command: ")
        if command.lower() == "stop":
            print("Stopping rover...")
            pi.set_state(RoverState.IDLE)
            break

    rover_thread.join()
    print("Rover has stopped.")
