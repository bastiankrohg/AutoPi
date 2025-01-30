import socket
import json
import threading

class VisionCoralListener:
    def __init__(self, listen_ip="0.0.0.0", listen_port=60010, buffer_size=4096):
        """
        Initializes the UDP listener for receiving vision inference results in a background thread.

        :param listen_ip: IP address to bind the listener.
        :param listen_port: Port to listen for incoming UDP packets.
        :param buffer_size: Maximum size of incoming UDP packets.
        """
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.buffer_size = buffer_size
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.listen_ip, self.listen_port))

        self.running = False  # Control flag for the listener thread
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.latest_detections = {"detections": []}  # Dictionary format for easier JSON conversion
        self.lock = threading.Lock()  # Ensures thread-safe access

        print(f"🟢 VisionCoral UDP Listener started on {self.listen_ip}:{self.listen_port}")

    def _process_inference_data(self, data):
        """
        Processes incoming inference results and stores them in a thread-safe manner.

        :param data: JSON-encoded inference result.
        """
        try:
            detections = json.loads(data)
            
            with self.lock:
                self.latest_detections["detections"] = detections  # Update stored inference data

            if not detections:
                print("[INFO] No objects detected.")
                return

            print("[INFO] Received Inference Data:")
            for obj in detections:
                class_id = obj.get("class_id", "Unknown")
                bbox = obj.get("bbox", [])
                score = obj.get("score", 0.0)
                label = obj.get("label", "Unknown")

                print(f" - Detected: {label} (ID: {class_id})")
                print(f"   - Bounding Box: {bbox}")
                print(f"   - Confidence: {score:.2f}\n")

        except json.JSONDecodeError:
            print("[ERROR] Failed to decode JSON from incoming data.")

    def _run(self):
        """
        Internal method that continuously listens for UDP inference results.
        """
        print("🟢 VisionCoral Listener thread running...")
        self.running = True
        
        while self.running:
            try:
                data, addr = self.sock.recvfrom(self.buffer_size)
                print(f"[INFO] Data received from {addr}")
                self._process_inference_data(data.decode("utf-8"))
            except Exception as e:
                print(f"[ERROR] VisionCoral Listener Error: {e}")

    def start(self):
        """
        Starts the UDP listener in a background thread.
        """
        if not self.thread.is_alive():
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            print("🟢 VisionCoral Listener thread started.")

    def stop(self):
        """
        Stops the UDP listener gracefully.
        """
        self.running = False
        self.thread.join()
        self.sock.close()
        print("🔴 VisionCoral Listener stopped.")

    def get_latest_detections(self):
        """
        Returns the latest inference results as a dictionary (thread-safe).

        :return: Dictionary containing detected objects.
        """
        with self.lock:
            return self.latest_detections.copy()  # Return a copy to prevent race conditions

    def get_latest_detections_json(self):
        """
        Returns the latest inference results as a JSON string (thread-safe).

        :return: JSON string containing detected objects.
        """
        with self.lock:
            return json.dumps(self.latest_detections)

if __name__ == "__main__":
    listener = VisionCoralListener()
    listener.start()

    try:
        while True:
            print(f"📡 Latest Detections JSON: {listener.get_latest_detections_json()}")
    except KeyboardInterrupt:
        print("\nStopping VisionCoral Listener...")
        listener.stop()