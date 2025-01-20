import rover

class MotorController:
    def __init__(self):
        """Initializes the motor controller for the rover."""
        rover.init(brightness=0)  # Initialize hardware

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

class SensorController:
    def __init__(self):
        """Initializes the sensor controller for the rover."""
        rover.init(brightness=0)  # Ensure hardware is initialized

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

    def navigate_around_obstacle(self):
        """Example method for avoiding obstacles."""
        distance = self.sensor_controller.get_distance()
        if distance < 20:
            self.motor_controller.stop()
            self.motor_controller.turn_right(50)
            time.sleep(1)
            self.motor_controller.drive_forward(50)