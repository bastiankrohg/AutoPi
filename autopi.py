from email import message
import threading
import time
import datetime
import signal
import sys
import os
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

import logging
import os

# Define log file
LOG_FILE = "rover.log"
LOG_FORMAT = "%(asctime)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s"


# Setup logging
logging.basicConfig(
    level=logging.DEBUG,  # Log everything (INFO, DEBUG, ERROR, etc.)
    format=LOG_FORMAT,
    handlers=[
        logging.FileHandler(LOG_FILE, mode="w"),  # Overwrites file on restart
    ],
)

logging.info("🚀 AutoPi logging initialized.")

# State Machine States
class RoverState:
    IDLE = "Idle"
    EXPLORING = "Exploring"
    AVOIDING_OBSTACLE = "AvoidingObstacle"
    PURSUING_RESOURCE = "PursuingResource"
    SIMULATING = "Simulating"

class AutoPi:
    def __init__(self, telemetry_ip, telemetry_port, debug_mode=False, path_type="straight_line", sim_mode=False):
        logging.info("Initializing AutoPi...")
        self.state = RoverState.IDLE

        self.rover = RoverHardware(brightness=0, PiBit=False)

        self.stop_event = threading.Event()  # Threading event for stopping execution

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
        
        self.distance_obstacle = -1
        # Message queue for VisionPi communication
        self.message_queue = queue.Queue()

        # Telemetry thread
        self.telemetry = Telemetry(telemetry_ip, telemetry_port, self.get_telemetry_data)
        self.telemetry.start()

        # MJPEG thread
        server_address = ('', 8080)  # Bind to all interfaces on port 8080
        self.mjpeg_server = MJPEGStreamServer(server_address, MJPEGStreamHandler)
        self.mjpeg_thread = threading.Thread(target=start_mjpeg_server, daemon=True)
        #self.mjpeg_thread = threading.Thread(target=self.mjpeg_server.start, daemon=True)
        self.mjpeg_thread.start()

        # Vision thread
        # Initialize VisionPi instance with the queue
        self.vision = VisionPi(
            rtsp_url="rtsp://example.com/stream",
            #path1="/home/pi/new_image.jpg",
            #path2="/home/pi/old_image.jpg",
            #path3="/home/pi/cropped_images",
            modelpath="/home/pi/models/beer_model.tflite",
            mode=0,
            message_queue=self.message_queue, 
            mjpeg_server=self.mjpeg_server
        )
        self.vision_thread = threading.Thread(target=self.vision.start, daemon=True)
        self.vision_thread.start()

        logging.info("AutoPi initialized.")
        
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
                    logging.info(f"[{datetime.datetime.now()}] AutoPi: Bottle detected. Generating path towards direction {direction:.2f} degrees.")
                    self.change_heading(int(direction))
                    self.state=RoverState.PURSUING_RESOURCE

                # Check ObstacleController alerts
                if self.obstacle_detector.is_alerted():
                    self.near_beer=1
                    logging.warning(f"[{datetime.datetime.now()}] Obstacle detected! Switching to avoiding obstacle mode.")
                    #self.set_state(RoverState.AVOIDING_OBSTACLE)
                
                    #self.avoid_obstacle()
                
    def set_state(self, new_state):
        with self.lock:
            logging.info(f"State change: {self.state} -> {new_state}")
            self.state = new_state

    def telemetry_loop(self):
        logging.info("Starting telemetry loop...")
        while not self.stop_event.is_set():
            proximity_alert = self.obstacle_detector.get_alert_source() if self.obstacle_detector.is_alerted() else None
            telemetry_data = {
                "position": self.map_center,
                "heading": self.heading,
                "battery_level": self.sensor_controller.get_battery_level(),
                "ultrasound_distance": self.sensor_controller.get_ultrasound_distance(),
                "state": self.state,  # Add current state to telemetry
                "proximity_alert": proximity_alert  # Add proximity indicator
            }
            logging.info(f"Telemetry data: {telemetry_data}")
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
            logging.info(f"Environment shifted to keep rover centered at {self.map_center}")        

    def change_heading(self, new_heading: int) -> int:
        """
        Rotates the rover towards the new heading using the shortest path.
        Turns in 5-degree steps for smoother control.

        Args:
            new_heading (int): The target heading in degrees (0-360).

        Returns:
            int: The updated heading after the turn.
        """
        current = self.heading % 360  # Normalize to 0-360
        new_heading = new_heading % 360  # Ensure target is within bounds

        # Compute both possible rotation angles
        delta_right = (new_heading - current) % 360  # Clockwise difference
        delta_left = (current - new_heading) % 360  # Counterclockwise difference

        # Choose the shortest turn direction
        if delta_right < delta_left:
            logging.info(f"Turning Right by {delta_right} degrees")
            while delta_right > 0:
                step = min(5, delta_right)  # Ensure we don't overshoot
                self.motor_controller.TurnRight(step)
                delta_right -= step
                self.heading = (self.heading + step) % 360  # Keep within [0,360]
                logging.info(f"Updated heading: {self.heading}°")
                time.sleep(0.1)  # Small delay for motor execution

        else:
            logging.info(f"Turning Left by {delta_left} degrees")
            while delta_left > 0:
                step = min(5, delta_left)
                self.motor_controller.TurnLeft(step)
                delta_left -= step
                self.heading = (self.heading - step) % 360  # Keep within [0,360]
                logging.info(f"Updated heading: {self.heading}°")
                time.sleep(0.1)  # Small delay for motor execution

        return self.heading

    def to_the_next_point(self): 
        if not self.current_path:
            logging.info("No more points in the path.")
            return

        next_position = self.current_path.pop(0)  # Get the next point
        shift_x = next_position[0] - self.map_center[0]
        shift_y = next_position[1] - self.map_center[1]

        if shift_x == 0 and shift_y == 0:
            logging.info("Already at target position.")
            return

        # Calculate target heading using atan2 (prevents division by zero)
        target_angle = math.degrees(math.atan2(shift_y, shift_x))
        
        self.change_heading(target_angle)
        if self.heading==target_angle: 
            logging.info("heading ok: %s", self.heading)
        # Update heading
        #self.heading = target_angle

        """
        # Normalize angles to be within 0-360 range
        current_heading = self.heading % 360
        target_angle = target_angle % 360

        # Compute the shortest rotation direction
        rotate_ang = (target_angle - current_heading + 180) % 360 - 180  # This ensures rotation is between -180 to 180

        if abs(rotate_ang) > 1:  # Avoid unnecessary small rotations
            if rotate_ang < 0:
                self.motor_controller.TurnLeft(abs(rotate_ang))
            else:
                self.motor_controller.TurnRight(abs(rotate_ang))
        """

        # Compute Euclidean distance to move forward
        distance = math.sqrt(shift_x**2 + shift_y**2) # in units
        distance_cm = 10 * distance
        logging.info(f"distance in units: {distance}, in cm: {distance_cm}")
        #print(f"Moving to {next_position}, turning {rotate_ang:.2f} degrees, then driving {distance:.2f} units forward.")

        # Move forward
        self.motor_controller.DriveForward(distance_cm) #distance in cm

        # Update internal map representation
        self.map_center = next_position  # Update rover's position
        logging.info(f"Next Position: {next_position}, New Heading: {self.heading} ")
        self.update_map()

    """           
    def update_map_obstacle(self,distance):
        logging.info("Updating map after avoiding obstacle")
        if self.current_path:
            next_position = self.current_path.pop(0)
            shift_x = next_position[0] - self.map_center[0]
            shift_y = next_position[1] - self.map_center[1]
            self.obstacles = {(x - shift_x, y - shift_y) for (x, y) in self.obstacles}
            self.map_center = next_position
            logging.info(f"Environment shifted to keep rover centered at {self.map_center}")
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
        logging.info("Entering exploration mode...")
        while self.state == RoverState.EXPLORING:
        
            self.current_path = self.generate_path()
            logging.info(f"Generated path: {self.current_path}")

            while self.current_path and self.state == RoverState.EXPLORING:
                self.to_the_next_point()
                #self.navigation_controller.follow_path(self.map_center, self.current_path)
                self.display_debug_info()
                time.sleep(0.1)
                """
                resource = self.vision.ultrasound_sensor() # TODO Fix?
                if resource:
                    logging.info(f"Something detected ahead: {resource}")
                    self.target_resource = resource
                    self.set_state(RoverState.PURSUING_RESOURCE)
                    return
                """
                    
    def simulation_mode(self):
        logging.info("Entering simulation mode...")
        while self.state == RoverState.SIMULATING:
            if not self.current_path:
                self.current_path = self.generate_path()
                logging.info(f"Generated path for simulation: {self.current_path}")

            if self.current_path:
                next_waypoint = self.current_path.pop(0)
                logging.info(f"Simulating move to {next_waypoint}")
                self.map_center = next_waypoint
                self.display_debug_info()
                time.sleep(1)  # Simulate movement delay

    def avoidance_mode(self):
        logging.info("Entering avoidance mode...")
        # Replan logic to avoid obstacle and rejoin original path
        while self.state == RoverState.AVOIDING_OBSTACLE:
            avoidance_goal = (self.map_center[0] + 2, self.map_center[1] + 2)  # Example: move diagonally to avoid
            avoidance_path = self.planner.plan(self.map_center, avoidance_goal, self.obstacles)
            logging.info(f"Avoidance path: {avoidance_path}")

            while avoidance_path:
                next_position = avoidance_path.pop(0)
                self.current_path.insert(0, next_position)  # Reinsert planned path ahead
                self.update_map()
                self.display_debug_info()

                if not self.obstacle_detector.is_alerted():
                    logging.info("Obstacle cleared. Returning to planned path.")
                    self.set_state(RoverState.EXPLORING)
                    return

            time.sleep(0.5)

    def pursuit_mode(self):
        logging.info("Entering pursuit mode...")
        while self.state == RoverState.PURSUING_RESOURCE:
            while self.near_beer == 0:
                time.sleep(0.1)
            self.motor_controller.stop()
            self.near_a_beer 
            self.state=RoverState.EXPLORING
    
    def display_dashboard(self):
        """Clears the terminal and displays real-time telemetry."""
        try:
            while not self.stop_event.is_set():
                os.system("clear")  # Clears terminal
                print("🚀 AutoPi Rover Dashboard\n")
                print(f"📍 Position: {self.current_position}")
                print(f"🧭 Heading: {self.heading}°")
                print(f"📡 Ultrasound Distance: {self.sensor_controller.last_ultrasound} m\n")
                print(f"📍 Path History: {self.current_path}")  # Show last 5 waypoints
                print("Press Ctrl+C to stop AutoPi.")
                time.sleep(2)  # Refresh every 2 seconds
        except KeyboardInterrupt:
            self.cleanup()

    def start(self):
        self.autopi_run()  # Demarre la logique du chemin principal
        
    def run(self):
        logging.info("Starting main control loop...")
        try:
            while not self.stop_event.is_set():
                if self.state == RoverState.EXPLORING:
                    self.exploration_mode()
                #elif self.state == RoverState.AVOIDING_OBSTACLE:
                #   self.avoidance_mode()
                elif self.state == RoverState.PURSUING_RESOURCE:
                    self.pursuit_mode()
                #elif self.state == RoverState.SIMULATING:
                #    self.simulation_mode()
                else:
                    logging.info("Rover is idle.")
                    time.sleep(0.1)
        except KeyboardInterrupt:
            # Fallback for unexpected interruptions
            logging.info("Running cleanup...")
            self.cleanup()
  
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C signal to stop the rover and clean up resources."""
        print("\nCtrl+C detected. Cleaning up and stopping the rover...")
        self.stop_event.set()  # Signal all threads to stop
        self.cleanup()
        print("Cleanup complete. Exiting.")
        sys.exit(0)

    def cleanup(self):
        """Perform cleanup tasks before shutting down."""
        try:
            print("Stopping the rover...")

            print("Stopping all services...")
            
            if self.motor_controller:
                print("[DEBUG] Stopping motor controller...")
                self.motor_controller.cleanup()
                print("[DEBUG] Motor controller stopped.")

            if self.telemetry:
                print("[DEBUG] Stopping telemetry...")
                self.telemetry.stop()
                print("[DEBUG] Telemetry stopped.")

            if self.vision:
                print("[DEBUG] Stopping vision processing...")
                self.vision.stop()
                print("[DEBUG] Vision processing stopped.")

            if self.mjpeg_server:
                print("[DEBUG] Stopping MJPEG server...")
                self.mjpeg_server.stop()
                print("[DEBUG] MJPEG server stopped.")

            print("All services stopped.")

        except Exception as e:
            print(f"Critical error during cleanup: {e}", exc_info=True)
            logging.error(f"Error during cleanup: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AutoPi Rover")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode to display mapping grid and path planning.")
    parser.add_argument("--path", type=str, default="expanding_square", help="Select path type: random_walk, spiral, zigzag, straight_line, sine_wave, expanding_square")
    parser.add_argument("--sim", action="store_true", help="Enable simulation mode.")
    parser.add_argument("--dashboard", action="store_true", help="Show the real-time telemetry dashboard.")
    args = parser.parse_args()

    TELEMETRY_IP = "127.0.0.1" 
    TELEMETRY_PORT = 50055  
    
    logging.info("Initializing rover...")
    try:
        pi = AutoPi(TELEMETRY_IP, TELEMETRY_PORT, debug_mode=args.debug, path_type=args.path, sim_mode=args.sim)
        signal.signal(signal.SIGINT, pi.signal_handler)

        # Start in appropriate mode
        initial_state = RoverState.SIMULATING if args.sim else RoverState.EXPLORING
        logging.info(f"Setting initial state to {initial_state}...")
        pi.set_state(initial_state)

        # Run the rover in a separate thread
        rover_thread = threading.Thread(target=pi.run, daemon=True)
        rover_thread.start()

        # 🚀 Run dashboard in foreground if --dashboard is set
        if args.dashboard:
            pi.display_dashboard()  # Blocks execution until Ctrl+C

        # If not running dashboard, keep main loop alive
        while not pi.stop_event.is_set():
            time.sleep(1)

        """
        # Command loop for user input
        while not self.stop_event.is_set():
            command = input("Enter command (type 'stop' to stop the rover): ").strip().lower()
            if command == "stop":
                logging.info("Stopping rover...")
                break
            else:
                logging.info(f"Unknown command: {command}. Type 'stop' to stop the rover.")
        """
    except Exception as e:
        pi.set_state(RoverState.IDLE)
        logging.error(f"Critical error during cleanup: {e}", exc_info=True)
        #logging.info(f"An error occurred: {e}")
    finally:
        pi.set_state(RoverState.IDLE)
        logging.info("Cleaning up...")
        pi.cleanup()  # Ensure the rover stops before exiting
        rover_thread.join()
        print("Rover has stopped.")
        logging.info("Rover has stopped.")