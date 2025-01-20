import threading
import time
from queue import SimpleQueue

class ObstacleDetector:
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

    def _check_sensors(self):
        """Periodically checks the sensors and signals an obstacle if necessary."""
        while self.running:
            ultrasound_distance = self.ultrasound_sensor()
            camera_detection = self.camera_sensor()

            if ultrasound_distance < 10:  # Threshold for obstacle detection in cm
                self.queue.put("Ultrasound detected obstacle")
                self.obstacle_alert.set()
            elif camera_detection:
                self.queue.put("Camera detected obstacle")
                self.obstacle_alert.set()

            time.sleep(self.detection_interval)

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

# Mock sensor functions for testing
def mock_ultrasound_sensor():
    """Simulates an ultrasound sensor returning random distances."""
    import random
    return random.randint(5, 20)  # Random distance between 5cm and 20cm

def mock_camera_sensor():
    """Simulates a camera detecting an obstacle occasionally."""
    import random
    return random.choice([True, False, False, False])  # 25% chance to detect

if __name__ == "__main__":
    detector = ObstacleDetector(mock_ultrasound_sensor, mock_camera_sensor)
    detector.start()

    try:
        while True:
            if detector.is_alerted():
                print("Obstacle detected! Source:", detector.get_alert_source())
                detector.reset_alert()
            time.sleep(0.1)
    except KeyboardInterrupt:
        detector.stop()
