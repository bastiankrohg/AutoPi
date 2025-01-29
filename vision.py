import socket
import json

class VisionCoralListener:
    def __init__(self, listen_ip="0.0.0.0", listen_port=60010, buffer_size=4096):
        """
        Initializes the UDP listener for receiving vision inference results.

        :param listen_ip: IP address to bind the listener.
        :param listen_port: Port to listen for incoming UDP packets.
        :param buffer_size: Maximum size of incoming UDP packets.
        """
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.buffer_size = buffer_size

        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.listen_ip, self.listen_port))

        print(f"VisionCoral UDP Listener started on {self.listen_ip}:{self.listen_port}")

    def process_inference_data(self, data):
        """
        Processes incoming inference results.

        :param data: JSON-encoded inference result.
        """
        try:
            detections = json.loads(data)
            
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

    def run(self):
        """
        Starts the UDP listener loop.
        """
        try:
            while True:
                data, addr = self.sock.recvfrom(self.buffer_size)
                print(f"[INFO] Data received from {addr}")

                self.process_inference_data(data.decode("utf-8"))

        except KeyboardInterrupt:
            print("\n[INFO] Stopping VisionCoral Listener...")
        finally:
            self.sock.close()
            print("[INFO] UDP Listener shut down.")

if __name__ == "__main__":
    listener = VisionCoralListener()
    listener.run()