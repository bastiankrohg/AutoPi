from email import message
import threading
import time
import datetime
import socket
import json
import platform
import argparse
import queue
import math
import datetime

from planning import AStarPlanner
from telemetry import Telemetry
from mjpeg import MJPEGStreamHandler, MJPEGStreamServer, start_mjpeg_server

from path import (
    generate_expanding_square_path, 
    generate_random_walk_path, 
    generate_sine_wave_path, 
    generate_spiral_pattern, 
    generate_zigzag_pattern, 
    generate_straight_line_path
)
from vision_pi import VisionPi

if platform.system() == "Linux":
    from controllers import MotorController, SensorController, NavigationController, RoverHardware, ObstacleController
else:
    from dummy import MotorController, SensorController, NavigationController


# State Machine States
class RoverState:
    IDLE = "Idle"
    EXPLORING = "Exploring"
    AVOIDING_OBSTACLE = "AvoidingObstacle"
    PURSUING_RESOURCE = "PursuingResource"
    SIMULATING = "Simulating"

class AutoPi:
    def __init__(self, telemetry_ip, telemetry_port, debug_mode=False, path_type="straight_line", sim_mode=False):
        print("Initializing AutoPi...")
        self.state = RoverState.IDLE

        self.rover = RoverHardware(brightness=0, PiBit=False)

        self.current_position=[0,0]
        self.motor_controller = MotorController(self.rover)
        self.sensor_controller = SensorController(self.rover)
        self.navigation_controller = NavigationController(motor_controller=self.motor_controller, sensor_controller=self.sensor_controller)
        
        self.current_path = []  # Exploration path
        self.target_resource = None
        self.lock = threading.Lock()
        self.map_center = (0, 0)  # Rover's position in the local map
        self.grid_size = 20  # Define grid size for local map
        self.obstacles = set()
        self.planner = AStarPlanner(self.grid_size)
        self.debug_mode = debug_mode
        self.sim_mode = sim_mode
        self.heading = 0  # Default heading is North, from 0 to 360, rover turn by 5 degrees
        self.near_beer=0 # 0 - not near a beer 1 - near a beer
        # Path selection
        self.path_type = path_type 
        # Obstacle Detector
        self.obstacle_detector = ObstacleController(self.sensor_controller.get_ultrasound_distance)
        self.obstacle_detector.start()
 
        #set_controller
        self.speed_angle_left=self.motor_controller.Calibrate_turn_left(50)
        self.speed_angle_right=self.motor_controller.Calibrate_turn_right(50)
        self.speed_forward=self.motor_controller.calibration_forward(80)
        
        # initialise obstacle_detector
        self.obstacle_detector = ObstacleController(self.sensor_controller.get_ultrasound_distance)
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
            mode=0,
            message_queue=self.message_queue
        )

        # Vision thread
        self.vision_thread = threading.Thread(target=self.vision.run, daemon=True)
        self.vision_thread.start()

        # Telemetry
        self.telemetry = Telemetry(telemetry_ip, telemetry_port, self.get_telemetry_data)
        self.telemetry.start()

        self.mjpeg_server = MJPEGStreamServer(('', 8080), MJPEGStreamHandler)
        self.mjpeg_thread = threading.Thread(target=start_mjpeg_server, kwargs={"port": 8080}, daemon=True)
        self.mjpeg_thread.start()

        print("AutoPi initialized.")
        
        # Draw initial map if in debug mode
        if self.debug_mode:
            self.display_debug_info()

    def handle_messages(self):
            """
            Handle messages from VisionPi and ObstacleController.
            """
            # Check VisionPi messages
            while not self.message_queue.empty():
                message = self.message_queue.get()
                if message["type"] == "bottle_detected":
                    direction = message["direction"]
                    print(f"[{datetime.now()}] AutoPi: Bottle detected. Generating path towards direction {direction:.2f} degrees.")
                    self.change_heading(int(direction))
                    self.state=RoverState.PURSUING_RESOURCE

                # Check ObstacleController alerts
                if self.obstacle_detector.is_alerted():
                    self.near_beer=1
                    print(f"[{datetime.now()}] Obstacle detected! Switching to avoiding obstacle mode.")
                    #self.set_state(RoverState.AVOIDING_OBSTACLE)
                
                    #self.avoid_obstacle()
                
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
    def get_telemetry_data(self):
        """Gathers telemetry data from the rover."""
        proximity_alert = self.obstacle_detector.get_alert_source() if self.obstacle_detector.is_alerted() else None
        return {
            "position": self.map_center,
            "heading": self.heading,
            "battery_level": self.sensor_controller.get_battery_level(),
            "ultrasound_distance": self.sensor_controller.get_ultrasound_distance(),
            "state": self.state,
            "proximity_alert": proximity_alert,
            "system_metrics": self.telemetry.get_system_state()
        }

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

    def update_map(self):
        print("Updating map...")
        if self.current_path:
            next_position = self.current_path.pop(0)
            shift_x = next_position[0] - self.map_center[0]
            shift_y = next_position[1] - self.map_center[1]
            self.obstacles = {(x - shift_x, y - shift_y) for (x, y) in self.obstacles}
            self.map_center = next_position
            print(f"Environment shifted to keep rover centered at {self.map_center}")        

    def change_heading(self, new_heading=None) -> int:
        if new_heading is None:
            print("new_heading is None! Exiting function.")
            return self.heading

        print(f"Changing heading from {self.heading}° to {new_heading}°")

        current = self.heading
        delta = new_heading - current

        if delta > 0:  # Turn Right
            print(f"Turning Right by {delta} degrees")
            while delta > 0:
                step = min(5, delta)  # Ensure we don't overshoot
                self.motor_controller.TurnRight(step)
                delta -= step
                self.heading += step  # Increase heading
                print(f"Updated heading: {self.heading}°")

        elif delta < 0:  # Turn Left
            print(f"Turning Left by {-delta} degrees")
            while delta < 0:
                step = min(5, abs(delta))
                self.motor_controller.TurnLeft(step)
                delta += step  # Move towards 0
                self.heading -= step  # Decrease heading
                print(f"Updated heading: {self.heading}°")

        else:  # No change needed
            print("Keep current heading. No turn required.")

        return self.heading

    def to_the_next_point(self): 
        next_position=self.current_path.pop(0)
        shift_x = next_position[0] - self.map_center[0]
        shift_y = next_position[1] - self.map_center[1]
        ang= int( math.atan(shift_y/shift_x) )
        rotate_ang=ang-self.heading
        number_rotations=int(abs( math.degrees(rotate_ang) /5) )
        if (rotate_ang<0) :
            for i in range(number_rotations): 
                self.motor_controller.TurnLeft(5)
        else :
            for i in range(number_rotations): 
                self.motor_controller.TurnRight(5)
        self.heading+=math.degrees(ang) % 360
        self.motor_controller.Driveforward()
        self.update_map()
    """           
    def update_map_obstacle(self,distance):
        print("Updating map after avoiding obstacle")
        if self.current_path:
            next_position = self.current_path.pop(0)
            shift_x = next_position[0] - self.map_center[0]
            shift_y = next_position[1] - self.map_center[1]
            self.obstacles = {(x - shift_x, y - shift_y) for (x, y) in self.obstacles}
            self.map_center = next_position
            print(f"Environment shifted to keep rover centered at {self.map_center}")
    """
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
        
            self.current_path = self.generate_path()
            print(f"Generated path: {self.current_path}")

            while self.current_path and self.state == RoverState.EXPLORING:
                self.to_the_next_point()
                self.navigation_controller.follow_path(self.map_center, self.current_path)
                self.display_debug_info()
                """
                resource = self.obstacle_detector.ultrasound_sensor() # TODO Fix?
                if resource:
                    print(f"Something detected ahead: {resource}")
                    self.target_resource = resource
                    self.set_state(RoverState.PURSUING_RESOURCE)
                    return
                """
                time.sleep(0.1)
                
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
            while self.near_beer == 0:
                time.sleep(0.1)
            self.motor_controller.stop()
            self.near_a_beer 
            self.state=RoverState.EXPLORING
        

    def start(self):
        # Demarre les threads pour autopi et vision_pi
        vision_thread = threading.Thread(target=self.vision_pi_run)
        vision_thread.start()

        self.autopi_run()  # Demarre la logique du chemin principal

        vision_thread.join()  # Attendre que le thread de vision se termine
        
    def run(self):
        print("Starting main control loop...")
        try:
            while True:
                if self.state == RoverState.EXPLORING:
                    self.exploration_mode()
                #elif self.state == RoverState.AVOIDING_OBSTACLE:
                #   self.avoidance_mode()
                elif self.state == RoverState.PURSUING_RESOURCE:
                    self.pursuit_mode()
                #elif self.state == RoverState.SIMULATING:
                #    self.simulation_mode()
                else:
                    print("Rover is idle.")
                    time.sleep(0.1)
        except KeyboardInterrupt:
            # Fallback for unexpected interruptions
            self.cleanup()
  
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C signal to stop the rover and clean up resources."""
        print("Ctrl+C detected. Cleaning up...")
        self.cleanup()
        print("Cleanup complete. Exiting.")
        exit(0)

    def cleanup(self):
        """Perform cleanup tasks before shutting down."""
        try:
            print("Stopping the rover...")
            self.motor_controller.cleanup()  # Stop the motors

            self.telemetry.stop()

            self.mjpeg_server.stop()
            self.mjpeg_thread.join()

            self.vision_thread.join()

        except Exception as e:
            print(f"Error during cleanup: {e}")





if __name__ == "__main__":
    import signal

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\nCtrl+C detected. Cleaning up and stopping the rover...")
        pi.cleanup()  # Ensure the rover stops
        exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser(description="AutoPi Rover")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode to display mapping grid and path planning.")
    parser.add_argument("--path", type=str, default="expanding_square", help="Select path type: random_walk, spiral, zigzag, straight_line, sine_wave, expanding_square")
    parser.add_argument("--sim", action="store_true", help="Enable simulation mode.")
    args = parser.parse_args()

    TELEMETRY_IP = "127.0.0.1"  # Replace with actual IP address
    TELEMETRY_PORT = 50055  # Replace with actual port

    print("Initializing rover...")
    try:
        pi = AutoPi(TELEMETRY_IP, TELEMETRY_PORT, debug_mode=args.debug, path_type=args.path, sim_mode=args.sim)

        # Start in appropriate mode
        initial_state = RoverState.SIMULATING if args.sim else RoverState.EXPLORING
        print(f"Setting initial state to {initial_state}...")
        pi.set_state(initial_state)

        # Run the rover in a separate thread
        rover_thread = threading.Thread(target=pi.run)
        rover_thread.start()

        # Command loop for user input
        while True:
            command = input("Enter command (type 'stop' to stop the rover): ").strip().lower()
            if command == "stop":
                print("Stopping rover...")
                pi.set_state(RoverState.IDLE)
                break
            else:
                print(f"Unknown command: {command}. Type 'stop' to stop the rover.")

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Cleaning up...")
        pi.cleanup()  # Ensure the rover stops before exiting
        rover_thread.join()
        print("Rover has stopped.")