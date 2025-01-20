
# Mock implementations for testing on non-Linux systems
class MotorController:
    def move(self, direction, speed):
        print(f"[Mock] Moving {direction} at speed {speed}")

class SensorController:
    def get_position(self):
        return {"x": 0, "y": 0}

    def get_heading(self):
        return 0

    def get_battery_level(self):
        return 100

    def get_ultrasound_distance(self):
        return 10

    def check_for_obstacles(self):
        return False

    def detect_resource(self):
        return None

class NavigationController:
    def follow_path(self, path):
        print(f"[Mock] Following path: {path}")

    def navigate_around_obstacle(self):
        print("[Mock] Navigating around obstacle")
        return True

    def move_to(self, target):
        print(f"[Mock] Moving to target: {target}")
        return True