from rover import rover

import time
import math
import threading
from queue import SimpleQueue
#from obstacle import ObstacleController

class RoverHardware:
    """Singleton-like class for shared rover hardware resources."""
    def __init__(self, brightness=0, PiBit=False):
        # Initialize the rover hardware once
        print("Initializing RoverHardware...")
        rover.init(brightness=brightness, PiBit=PiBit)
        print("RoverHardware initialized.")


class MotorController:
    def __init__(self, rover=None):
        """Initializes the motor controller for the rover."""
        rover.init(brightness=0)  # Initialize hardware
        self.angular_speed_right=0.0
        self.angular_speed_left=0.0
        self.forward_speed=0.0

    def drive_forward(self, speed):
        """Drives the rover forward at a specified speed."""
        rover.forward(speed)

    def drive_backward(self, speed):
        """Drives the rover backward at a specified speed."""
        rover.reverse(speed)

    def turn_left(self, speed):
        """Turns the rover left by spinning the wheels in opposite directions."""
        rover.spinLeft(speed)

    def turn_right(self, speed):
        """Turns the rover right by spinning the wheels in opposite directions."""
        rover.spinRight(speed)

    def stop(self):
        """Stops the rover's motors."""
        rover.stop()

    def cleanup(self):
        """Runs cleanup by stopping rover and cleaning gpio"""
        rover.cleanup()

    def turn(self, angle):
            """
            Turns the rover by the specified angle.

            Parameters:
            - angle (float): The angle in degrees. Positive for right, negative for left.
            """
            if angle > 0:
                print(f"Turning right by {angle} degrees")
                self.turn_right(min(abs(angle), 100))  # Limit max speed/angle
            elif angle < 0:
                print(f"Turning left by {angle} degrees")
                self.turn_left(min(abs(angle), 100))  # Limit max speed/angle
            time.sleep(abs(angle) / 50)  # Simulate time proportional to angle
            self.stop()  # Ensure the rover stops after turning
                    
    def Calibrate_turn_right(self, speed) :
            print(f"Rover turning Right at speed 50")
            self.turn_right(50)
            time.sleep(2)
            self.stop()
            angle= float (input ("angle parcouru par le rover :"))
            speed_ang=angle/5                     
            print (f"angular speed :{speed_ang}") 
            self.angular_speed_right =speed_ang

    def Calibrate_turn_left(self, speed) :
            print(f"Rover turning Right at speed {speed}")
            self.turn_left(50)
            time.sleep(2)
            self.stop()
            angle= float (input ("angle parcouru par le rover :"))
            speed_ang=angle/5                     
            print (f"angular speed :{speed_ang}") 
            self.angular_speed_left =speed_ang
 
    #calibration forward rover
    def calibration_forward(self,speed) :
        self.drive_forward(speed)
        time.sleep(5)
        self.stop()
        distance= float (input ("distance parcouru par le rover :"))
        speed_forward=distance/3                   
        print (f"speed :{speed_forward}") 
        self.forward_speed = speed_forward
        
        
    def TurnRight(self, angle):  
    
        #angle= float (input ("angle a parcourir par le rover :"))  
        timeOFF =angle/self.angular_speed_right if self.angular_speed_right else 1
        self.turn_right(50)
        time.sleep(timeOFF)
        self.stop()
        print (f"rover has turn 5 degrees to the right") 
        
    def TurnLeft(self, angle):  
    
        #angle= float (input ("angle a parcourir par le rover :"))  
        timeOFF =angle/self.angular_speed_left if self.angular_speed_right else 1
        self.turn_left(50)
        time.sleep(timeOFF)
        self.stop()
        print (f"rover has turn 5 degrees to the left") 

    def TurnRight5(self):  
        
        #angle= float (input ("angle a parcourir par le rover :"))  
        timeOFF =5/self.angular_speed_right if self.angular_speed_right else 1
        self.turn_right(50)
        time.sleep(timeOFF)
        self.stop()
        print (f"rover has turn 5 degrees to the right") 
        
    def TurnLeft5(self):  
    
        #angle= float (input ("angle a parcourir par le rover :"))  
        timeOFF =5/self.angular_speed_left if self.angular_speed_right else 1
        self.turn_left(50)
        time.sleep(timeOFF)
        self.stop()
        print (f"rover has turn 5 degrees to the left") 
        
    def Driveforward(self):
            
        timeOFF =10/self.forward_speed
        self.drive_forward(80)
        time.sleep(timeOFF)
        self.stop()
        print (f"rover has drive 10 cm ") 
        
    #a voir
    def turn(self, angle):
        """
        Turns the rover by the specified angle.
        Parameters:
        - angle (float): The angle in degrees. Positive for right, negative for left.
        """
        if angle > 0:
            print(f"Turning right by {angle} degrees")
            self.turn_right(min(abs(angle), 100))  # Limit max speed/angle
        elif angle < 0:
            print(f"Turning left by {angle} degrees")
            self.turn_left(min(abs(angle), 100))  # Limit max speed/angle
        time.sleep(abs(angle) / 50)  # Simulate time proportional to angle
        self.stop()  # Ensure the rover stops after turning



class SensorController:
    def __init__(self, rover=None):
        """Initializes the sensor controller using the shared rover hardware."""
        print("SensorController initialized.")

    def get_ultrasound_distance(self):
        """Fetches the distance from the ultrasonic sensor."""
        return rover.getDistance()

    def get_battery_level(self):
        """Fetches the current battery level."""
        print("Battery function not available")
        return 100

class ObstacleController:
    def __init__(self, ultrasound_sensor, camera_sensor, detection_interval=0.1):
        """
        ObstacleDetector monitors the sensors and triggers an alert if an obstacle is detected.

        Args:
            ultrasound_sensor: A callable object/function that returns distance to nearest object.
            camera_sensor: A callable object/function that performs object detection.
            detection_interval: Time in seconds between sensor checks.
        """
        self.ultrasound_sensor = ultrasound_sensor
        self.camera_sensor = camera_sensor
        self.detection_interval = detection_interval
        self.obstacle_alert = threading.Event()
        self.queue = SimpleQueue()
        self.running = False
        
        self.speed_angle_right=0.0
        self.speed_angle_left=0.0
        self.distance_obstacle=0.0
        
        
        

    def _check_sensors(self):
        """Periodically checks the sensors and signals an obstacle if necessary."""

        ultrasound_distance = self.ultrasound_sensor()
        camera_detection = self.camera_sensor()

        if ultrasound_distance < 10:  # Threshold for obstacle detection in cm
            self.queue.put(("Ultrasound detected obstacle",ultrasound_distance))
            self.obstacle_alert.set()
        elif camera_detection:
            self.queue.put("Camera detected obstacle")
            self.obstacle_alert.set()


    def start(self):
        """Starts the obstacle detection thread."""
        if not self.running:
            self.running = True
            self.detection_thread = threading.Thread(target=self._check_sensors, daemon=True)
            self.detection_thread.start()
            print("ObstacleDetector started.")

    def stop(self):
        """Stops the obstacle detection thread."""
        self.running = False
        if hasattr(self, 'detection_thread'):
            self.detection_thread.join()
            print("ObstacleDetector stopped.")

    def reset_alert(self):
        """Resets the obstacle alert."""
        self.obstacle_alert.clear()

    def is_alerted(self):
        """Checks if an obstacle alert is active."""
        return self.obstacle_alert.is_set()

    def get_alert_source(self):
        """Returns the source of the latest obstacle alert, if any."""
        if not self.queue.empty():
            return self.queue.get()
        return None
    
    def avoid_obstacle(self):
        distance_contournement=self.distance_obstacle/math.cos(45)
        self.rover.TurnRight(45,self.speed_angle_right)
        self.rover.drive_forward(50) ##calibration du drive_forward a faire
        time.sleep(5)
        self.rover.TurnLeft(45,self.speed_angle_left)
        self.rover.drive_forward(50)
        time.sleep(5)
        print(f"the rover has drive a distance {distance_contournement*2} ")
        return distance_contournement*2
        
    def run(self) :
        self.start()
        while self.running: 
            self._check_sensors()
            #if self.is_alerted() :
             #   alert = self.get_alert_source()
              #  distance_parcourue=self.avoid_obstacle(alert[1])
               # self.reset(alert)


class NavigationController:
    def __init__(self, motor_controller, sensor_controller):
        """Initializes the navigation controller with motor and sensor controllers."""
        self.motor_controller = motor_controller
        self.sensor_controller = sensor_controller
        self.current_heading = 0  # Heading in degrees, 0 = North
        self.obstaclecontroller = ObstacleController()#à remplir
        self.current_heading = 0  # Heading in degrees, 0 = North

    def navigate_around_obstacle(self):
        """Example method for avoiding obstacles."""
        distance = self.sensor_controller.get_ultrasound_distance()
        if distance < 20:
            print("Obstacle detected! Navigating around it...")
            self.motor_controller.stop()
            self.motor_controller.turn_right(50)
            time.sleep(1)
            self.motor_controller.drive_forward(50)

    def calculate_turn_angle(self, current_position, next_waypoint):
        """
        Calculate the angle the rover needs to turn to face the next waypoint.
        Parameters:
        - current_position (tuple): The current (x, y) position of the rover.
        - next_waypoint (tuple): The target (x, y) position.
        Returns:
        - float: The angle to turn in degrees.
        """
        dx = next_waypoint[0] - current_position[0]
        dy = next_waypoint[1] - current_position[1]
        target_angle = math.degrees(math.atan2(dy, dx))
        turn_angle = target_angle - self.current_heading

        # Normalize the angle to [-180, 180]
        turn_angle = (turn_angle + 180) % 360 - 180
        return turn_angle

    def drive_to_waypoint(self, current_position, next_waypoint):
        """
        Drive the rover to the next waypoint.
        Parameters:
        - current_position (tuple): The current (x, y) position of the rover.
        - next_waypoint (tuple): The target (x, y) position.
        """
        # Calculate the turn angle
        turn_angle = self.calculate_turn_angle(current_position, next_waypoint)
        distance = math.sqrt((next_waypoint[0] - current_position[0])**2 + (next_waypoint[1] - current_position[1])**2)

        # Turn the rover
        if abs(turn_angle) > 1:  # Threshold to avoid unnecessary small turns
            print(f"Turning {turn_angle:.2f} degrees to face waypoint {next_waypoint}")
            self.motor_controller.turn(turn_angle)
            self.current_heading += turn_angle
            self.current_heading %= 360  # Keep heading within [0, 360)

        # Drive forward
        print(f"Driving {distance:.2f} units forward to waypoint {next_waypoint}")
        self.motor_controller.drive_forward(distance)

    def follow_path(self, current_position, path):
        """
        Follow a sequence of waypoints.
        Parameters:
        - current_position (tuple): The starting (x, y) position of the rover.
        - path (list): A list of waypoints [(x1, y1), (x2, y2), ...].
        """
        for waypoint in path:
            self.drive_to_waypoint(current_position, waypoint)
            current_position = waypoint  # Update current position
            self.motor_controller.drive_forward(50)
            
    