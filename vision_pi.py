
import signal
import sys
import os
from typing import Any
import rover
import time
import datetime
#import tensorflow.lite as tflite
import threading
import queue
import cv2
import numpy as np
import json

import logging
# Use existing logger
logger = logging.getLogger(__name__)

class VisionPi:
    def __init__(self,rtsp_url,path1,path2,path3,modelpath,mode,message_queue,mjpeg):
        self.rtsp_url= rtsp_url
        self.new_image = path1
        self.old_image = path2
        self.cropped_image = path3
        self.modelpath = modelpath
        self.state = -1
        self.direction = 0.00
        self.message_queue = message_queue
        self.mode=mode     #0 - autonomy; 1 - hybride; or 2 - with coral
        self.mjpeg=mjpeg

        self.running = False
        self.thread = None
    
    ## functions for the Tensorflow model preparation and utilisation
    """
    def load_tflite_model(self):
        #Load the TensorFlow Lite model.
        logger.info(f"[{datetime.datetime.now()}] Loading TensorFlow Lite model from {self.modelpath}...")
        interpreter = tflite.Interpreter(model_path=self.modelpath)
        interpreter.allocate_tensors()
        return interpreter

    def run_tflite_model(self, interpreter, input_image):
        #Run inference with the TFLite model on an input image.
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        # Prepare the image for the model
        input_shape = input_details[0]['shape']
        input_data = cv2.resize(input_image, (input_shape[1], input_shape[2]))  # Resize to model input size
        input_data = input_data.astype('float32') / 255.0  # Normalize the image
        input_data = input_data.reshape(input_shape)  # Reshape to match input shape

        # Run inference
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()

        # Retrieve the results
        output_data = interpreter.get_tensor(output_details[0]['index'])
        return output_data

    """
    
    def start(self):
        """Starts the VisionPi in a separate thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()
            logger.info(f"VisionPi started.")

    def stop(self):
        """Stops the VisionPi listener."""
        self.running = False
        if self.thread:
            self.thread.join()
            logger.info("VisionPi listener stopped.")

    ##image processing
    def compare_images(self):
        """
        Compares two images using SSIM and returns the difference percentage.
        """
        # Convert images to grayscale
        gray1 = cv2.cvtColor(self.new_image, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(self.old_image, cv2.COLOR_BGR2GRAY)

        # Compute SSIM (Structural Similarity Index)
        score, _ = cv2.ssim(gray1, gray2, full=True)

        # Calculate difference percentage
        difference = 1 - score
        logger.info(f"[{datetime.datetime.now()}] SSIM score: {score:.4f}, Difference: {difference:.4f}")
        return difference

    def test_bottle_detection(self):
        """
        Simulated bottle detection function that processes the image.
        Replace with the actual implementation.
        """
        logger.info(f"[{datetime.datetime.now()}] Processing image using test_bottle_detection...")
    
        # Resize the image for processing
        image = cv2.resize(self.new_image, (640, 480))

        # Convert the image to HSV color space
        hsv = cv2.cvtColor(self.new_image, cv2.COLOR_BGR2HSV)

        # Adjusted HSV range for brown
        lower_brown = np.array([1, 60, 75])   # Lower bound for brown
        upper_brown = np.array([25, 255, 255])  # Upper bound for brown

        # Adjusted HSV range for dark blue
        lower_blue = np.array([90, 10, 30])  # Lower bound for dark blue
        upper_blue = np.array([150, 255, 150])  # Upper bound for dark blue

        # Create masks for both brown and dark blue
        mask_brown = cv2.inRange(hsv, lower_brown, upper_brown)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Combine the masks
        combined_mask = cv2.bitwise_or(mask_brown, mask_blue)

        # Morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask_cleaned = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)  # Close small gaps
        mask_cleaned = cv2.morphologyEx(mask_cleaned, cv2.MORPH_OPEN, kernel)  # Remove small noise

        # Detect contours from the cleaned mask
        contours, _ = cv2.findContours(mask_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Ensure the output directory exists
        if not os.path.exists(self.cropped_images):
            os.makedirs(self.cropped_images)

        # Store regions and their paths
        cropped_image_data = []  # To store (position, path) pairs

        for idx, cnt in enumerate(contours):
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = h / float(w)

            # Looser criteria: elongated shape, minimal size
            if aspect_ratio > 1.2 and h > 50 and cv2.contourArea(cnt) > 200:
                # Crop the detected region
                cropped_image = self.new_image[y:y + h, x:x + w]

                # Save the cropped image
                save_path = os.path.join(self.cropped_images, f"cropped_image_{idx}.png")
                cv2.imwrite(save_path, cropped_image)

                # Append position and path to the results
                cropped_image_data.append({
                    "position": (x, y, w, h),
                    "path": save_path
                })

        logger.info(f"[{datetime.datetime.now()}] Image processing complete. Cropped images saved to {self.cropped_images}.")
    
        # Return the array of cropped image data
        return cropped_image_data
     
    
    def process_cropped_image(self, cropped_image_data):
        """
        Process cropped images using a TensorFlow Lite model to detect bottles.
        Stops as soon as a bottle is detected and returns its position.
        """
        logger.info("hello")
        """
        interpreter = self.load_tflite_model(self.modelpath)

        for data in cropped_image_data:
            image_path = data["path"]
            position = data["position"]

            # Load the cropped image
            cropped_image = cv2.imread(image_path)

            # Run recognition using the model
            prediction = self.run_tflite_model(interpreter, cropped_image)

            # Assume the model returns probabilities for each class (e.g., [prob_non_beer, prob_beer])
            is_beer = prediction[0][1] > 0.5  # If the probability of "beer" is greater than 0.5

            if is_beer:
                logger.info(f"[{datetime.datetime.now()}] Beer detected at position: {position}")
                self.state=4
                return position  # Return immediately when a beer is detected

        logger.info(f"[{datetime.datetime.now()}] No beer detected in cropped images.")
        self.state=3
        return None  # Return None if no beer is found
        """

    def calcul_direction(self, beer_position):
        image_width = 640 #the stream of the camera is in 640*480p
        fov=62.2 # the camera as an a vision angle of 62.2 degres
        x, _, w, _ = beer_position
        beer_center_x = x + (w / 2)

        # Calculate the image's center x-coordinate
        image_center_x = image_width / 2

        # Calculate the pixel offset from the center of the image
        pixel_offset = beer_center_x - image_center_x

        # Calculate the degrees per pixel
        degrees_per_pixel = fov / image_width

        # Calculate the angle to turn
        self.direction = pixel_offset * degrees_per_pixel

    def one_image_processing(self):
        """
        Captures an image from the RTSP stream 
        Compares the current image with the previous one and performs actions based on the difference:
        - <10%: Does nothing.
        - 10%-70%: Processes the image with test_bottle_detectiona and process_cropped_image
        - >70%: Sends the image to Coral.
        Return the turn angle if a beer is found
        """
        self.code = -1
        cap = cv2.VideoCapture(self.rtsp_url)  # Open the RTSP stream

        if not cap.isOpened():
            logger.info("Error: Unable to open RTSP stream.")
            return

        previous_image = None

    
        # Capture a frame from the stream
        ret, frame = cap.read()
        if not ret:
            logger.info(f"[{datetime.datetime.now()}] Error: Unable to read frame from stream.")
            return

        # Save the current frame to the actual image path
        cv2.imwrite(self.new_image, frame)
        logger.info(f"[{datetime.datetime.now()}] Saved current frame as {self.path2}")

        # If there is a previous image, compare it with the current image
        if previous_image is not None:
            actual_image = cv2.imread(self.old_image)
            difference = VisionPi.compare_images(self.new_image, self.old_image)

            if difference < 0.1:
                logger.info(f"[{datetime.datetime.now()}] Images are too similar (<10%). Skipping.")
                self.state = 0
            elif 0.1 <= difference <= 0.7  or (0.1<=difference and self.mode==0):
                logger.info(f"[{datetime.datetime.now()}] Images are moderately different (10%-70%). Processing.")
                self.state = 1
                #cropped_images_with_locations =VisionPi.test_bottle_detection(frame)
                #location=VisionPi.process_cropped_image(self, cropped_images_with_locations)
                #if self.state == 3 : 
                #    VisionPi.calcul_direction(location)
              
            elif (difference > 0.7 and self.mode==2) or (0.1<=difference and self.mode==2):
                logger.info(f"[{datetime.datetime.now()}] Images are highly different (>70%). Sending to Coral.")
                self.state = 2
                
                #on attend un retour et on envoie a autopi la direction

        # Update the previous image
        cv2.imwrite(self.old_image, frame)
        previous_image = cv2.imread(self.new_image)
        logger.info(f"[{datetime.datetime.now()}] Updated previous frame as {self.path1}")

        # Wait for the specified interval before capturing the next frame
        time.sleep(self.interval)

        # Release the stream
        cap.release()
        logger.info(f"[{datetime.datetime.now()}] Stream closed.")

    def process_inference_data(self, data):
        """Processes incoming inference results from Coral."""
        try:
            detections = json.loads(data)
            return detections if detections else []
        except json.JSONDecodeError:
            logger.info("[ERROR] Failed to decode JSON from incoming data.")
            return []

    def run(self):
        try:
            while self.running:
                if self.mode == 0: # Pi only
                    logger.info("Mode: Pi only")
                    logger.info(f"[{datetime.datetime.now()}] Starting one image processing cycle...")
                    """
                    # Process a single cycle
                    self.run_1_time_hybrid()

                    # React based on the state
                    if self.state == 0:
                        logger.info(f"[{datetime.datetime.now()}] State 0: No action required.")
                        #wait until the coral get the 
                    elif self.state == 3:
                        logger.info(f"[{datetime.datetime.now()}] Alert: Bottle detected. Direction = {self.direction:.2f}°")
                        self.message_queue.put({"type": "bottle_detected", "direction": self.direction})
                    """
                elif self.mode == 1: # TODO Hybrid mode
                    # self.process_inference_data(data)
                    continue
                elif self.mode == 2: # TODO coral mode
                    # self.process_inference_data(data)
                    continue
                else: 
                    logger.info("Unexpected mode: ", self.mode)
                    break
                # Wait for 4 seconds before repeating
                time.sleep(4)
        except Exception as e:
            print(f"VisionPi Error: {e}")

if __name__ == "__main__":
    print("hello")



