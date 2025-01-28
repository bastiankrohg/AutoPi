import time
import math
from rover import rover

class MotorController:
    def __init__(self):
        """Initializes the motor controller for the rover."""
        rover.init(0, PiBit=False)

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
    def __init__(self):
        """Initializes the sensor controller for the rover."""
        rover.init(0, PiBit=False)

    def get_distance(self):
        """Fetches the distance from the ultrasonic sensor."""
        return rover.getDistance()

    def get_battery_level(self):
        """Fetches the current battery level."""
        return rover.getBattery()

class NavigationController:
    def __init__(self, motor_controller, sensor_controller):
        """Initializes the navigation controller with motor and sensor controllers."""
        self.motor_controller = motor_controller
        self.sensor_controller = sensor_controller
        self.current_heading = 0  # Heading in degrees, 0 = North

    def navigate_around_obstacle(self):
        """Example method for avoiding obstacles."""
        distance = self.sensor_controller.get_distance()
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
