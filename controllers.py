import rover
import time

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
        
    def Calibrate_turn_right(self) :
            print(f"Rover turning Right at speed {speed}")
            self.turn_right(50)
            time.sleep(5)
            self.stop()
            angle= float (input ("angle parcouru par le rover :"))
            speed_ang=angle/5                     
            print (f"angular speed :{speed_ang}") 
            return speed_ang

    def Calibrate_turn_left(self) :
            print(f"Rover turning Right at speed {speed}")
            self.turn_left(50)
            time.sleep(5)
            self.stop()
            angle= float (input ("angle parcouru par le rover :"))
            speed_ang=angle/5                     
            print (f"angular speed :{speed_ang}") 
            return speed_ang
        
    def TurnRight(self,angle,angular_speed):  
        
            #angle= float (input ("angle a parcourir par le rover :"))  
            timeOFF =angle/angular_speed 
            self.turn_right(50)
            time.sleep(timeOFF)
            print (f"rover has turn {angle} degrees") 
        
    def Turnleft(self,angle,angular_speed):  
        
            #angle= float (input ("angle a parcourir par le rover :"))  
            timeOFF =angle/angular_speed 
            self.turn_left(50)
            time.sleep(timeOFF)
            print (f"rover has turn {angle} degrees") 

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