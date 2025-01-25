import threading
import time
import socket
import json
import platform
import argparse
import queue
import math

from planning import AStarPlanner
from obstacle import ObstacleDetector
from vision_pi import VisionPi
from path import generate_expanding_square_path, generate_random_walk_path, generate_sine_wave_path, generate_spiral_pattern, generate_zigzag_pattern, generate_straight_line_path
from obstacle import ObstacleDetector
from controllers import MotorController

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
    def __init__(self, telemetry_ip, telemetry_port, debug_mode=False, path_type="straight_line"):
        print("Initializing AutoPi...")
        self.state = RoverState.IDLE
        self.motor_controller = MotorController()
        self.sensor_controller = SensorController()
        self.navigation_controller = NavigationController()
        self.current_path = []
        self.target_resource = None
        self.lock = threading.Lock()
        self.map_center = (0, 0)
        self.grid_size = 20
        self.obstacles = set()
        self.planner = AStarPlanner(self.grid_size)
        self.debug_mode = debug_mode
        self.heading = "N"
        
        #set_controller
        self.rover=MotorController()
        self.speed_angle_left=rover.Calibrate_turn_left(50)
        self.speed_angle_right=rover.Calibrate_turn_right(50)
        
        # initialise obstacle_detector
        self.obstacle_detector = ObstacleDetector(self.sensor_controller.get_ultrasound_distance, self.sensor_controller.detect_resource)
        self.distance_obstacle = -1
        # Message queue for VisionPi communication
        self.message_queue = queue.Queue()

        # Initialize VisionPi instance with the queue
        self.vision = VisionPi(
            rtsp_url="rtsp://example.com/stream",
            path1="/home/pi/new_image.jpg",
            path2="/home/pi/old_image.jpg",
            path3="/home/pi/cropped_images",
            modelpath="/home/pi/models/beer_model.tflite",
            message_queue=self.message_queue
        )

        # Vision thread
        self.vision_thread = threading.Thread(target=self.vision.run, daemon=True)
        self.vision_thread.start()

        # Telemetry
        self.telemetry_ip = telemetry_ip
        self.telemetry_port = telemetry_port
        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.telemetry_thread = threading.Thread(target=self.telemetry_loop, daemon=True)
        self.telemetry_thread.start()
        print("AutoPi initialized.")

    def avoid_obstacle(self):
        distance_contournement=self.distance_obstacle/math.cos(45)
        self.rover.TurnRight(45,self.speed_angle_right)
        self.rover.drive_forward(50) ##calibration du drive_forward à faire
        time.sleep(5)
        self.rover.TurnLeft(45,self.speed_angle_right)
        self.rover.drive_forward(50)
        time.sleep(5)
        print(f"the rover has drive a distance {self.distance_obstacle*2} ")




    def handle_messages(self):
            """
            Handle messages from VisionPi and ObstacleDetector.
            """
            # Check VisionPi messages
            while not self.message_queue.empty():
                message = self.message_queue.get()
                if message["type"] == "bottle_detected":
                    direction = message["direction"]
                    print(f"[{datetime.now()}] AutoPi: Bottle detected. Generating path towards direction {direction:.2f}°.")
                    self.generate_path_towards(direction)
                    self.set_state(RoverState.PURSUING_RESOURCE)  # Transition to pursuing resource state

            # Check ObstacleDetector alerts
            if self.obstacle_detector.is_alerted():
                print(f"[{datetime.now()}] Obstacle detected! Switching to avoiding obstacle mode.")
                self.set_state(RoverState.AVOIDING_OBSTACLE)
                self.distance_obstacle = self.sensor_controller.get_ultrasound_distance()
                self.avoid_obstacle()
                

    def generate_path_towards(self, direction):
        """
        Generate a path towards the specified direction.
        """
        print(f"Generating path towards direction {direction:.2f}°...")
        # Implement path generation logic here using `direction`

    def run(self):
        self.obstacle_detector.start()
        print("Starting main control loop...")
        while True:
            self.handle_messages()  # Check for messages from VisionPi
            if self.state == RoverState.EXPLORING:
                self.exploration_mode()
            elif self.state == RoverState.AVOIDING_OBSTACLE:
                self.avoidance_mode()
            elif self.state == RoverState.PURSUING_RESOURCE:
                self.pursuit_mode()
            else:
                print("Rover is idle.")
                time.sleep(0.1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AutoPi Rover")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode to display mapping grid and path planning.")
    parser.add_argument("--path", type=str, default="straight_line", help="Select path type: random_walk, spiral, zigzag, straight_line, sine_wave, expanding_square")
    args = parser.parse_args()

    TELEMETRY_IP = "127.0.0.1"  # Replace with actual IP address
    TELEMETRY_PORT = 50055  # Replace with actual port

    print("Initializing rover...")
    pi = AutoPi(TELEMETRY_IP, TELEMETRY_PORT, debug_mode=args.debug, path_type=args.path)

    # Start in exploring mode
    print("Setting initial state to EXPLORING...")
    pi.set_state(RoverState.EXPLORING)

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
