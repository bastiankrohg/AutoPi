
import signal
import sys
import os
from typing import Any
import time
import threading
import queue
import cv2
import numpy as np
import json
from listener import VisionCoralListener

import logging
# Use existing logger
logger = logging.getLogger(__name__)


class VisionPi:
    def __init__(self,rtsp_url,modelpath="/Autopi/model.onnx",mode=0,message_queue,mjpeg_server):
        self.mjpeg_server=mjpeg_server
        self.rtsp_url= rtsp_url
        self.new_image = None
        self.old_image = self.mjpeg_server.get_frame()
        self.modelpath = modelpath
        self.state = -1
        self.result= None
        self.difference =0
        self.direction = 0.00
        self.message_queue = message_queue
        
        self.mode=mode     #0 - autonomy; 1 - hybride; or 2 - with coral
        self.listener=VisionCoralListener()

        self.mjpeg_server
        self.running = False
        self.thread = None
        self.net= None
    
        ## functions for the Tensorflow model preparation and utilisation
    
    def load_yolov8nano_model(self):
        #Load the TensorFlow Lite model.
        print(f"[{datetime.datetime.now()}] Loading Yolov8nano model from {self.modelpath}...")
        # Charger le modele ONNX
        net2 = cv2.dnn.readNet("C:/Users/samsung/Downloads/model_test_quantized.onnx")
        net2.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)  # CPU
        net2.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.net =net

    def run_yolov8nano_model(self):
        image = cv2.imread(self.new_image)
        blob = cv2.dnn.blobFromImage(image, scalefactor=1/255.0, size=(640, 640), swapRB=True, crop=False)
        self.net.setInput(blob)
    
        # Mesurer le temps d'inférence
        start_time = time.time()
        output = self.net.forward()
        end_time = time.time()
        
    
        inference_time = end_time - start_time
        print(f"Inference Time: {inference_time:.4f} seconds")
        
        # Supprimer la premiere dimension inutile
        output = output[0]  # Devient (5, 8400)

        # Transposer pour faciliter la lecture chaque ligne devient une prediction
        output = output.T  # Devient (8400, 5)

        for detection in output:
            x_min, y_min, x_max, y_max, confidence = detection  # Extraction des 5 valeurs

        if confidence > 0.7:  # Filtrer les detections avec une confiance elevee
            output2=[(x_max-x_min)/2,confidence]
            print("bottle found")
            self.state=1
            return output2
        return None
    
    
    def start(self):
        """Starts the VisionPi in a separate thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()
            print(f"VisionPi started.")

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

        # Redimensionner l image precedente pour qu'elle corresponde a la taille de l'image actuelle
        if self.old_image.shape != self.new_image.shape:
            self.old_image = cv2.resize(self.old_image, (self.new_image.shape[1], self.new_image.shape[0]))

        # Convertir les images en niveaux de gris
        gray_previous = cv2.cvtColor(self.old_image, cv2.COLOR_BGR2GRAY)
        gray_current = cv2.cvtColor(self.new_image, cv2.COLOR_BGR2GRAY)

        # Calculer la difference absolue entre les deux images
        diff = cv2.absdiff(gray_previous, gray_current)

        # Calculer le taux de differentiation
        non_zero_count = np.count_nonzero(diff)
        total_pixels = diff.size
        difference_rate = (non_zero_count / total_pixels) 
        self.difference = difference_rate
        
        self.old_image=self.new_image

         logger.info(f"Taux de differentiation : {difference_rate:.2f}%")

    def test_bottle_detection(self) :
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

        # Count potential bottles
        bottle_count = 0

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = h / float(w)

            # Looser criteria: elongated shape, minimal size
            if aspect_ratio > 1.2 and h > 50 and cv2.contourArea(cnt) > 200:
                bottle_count += 1

         logger.info(f"[{datetime.datetime.now()}] Image processing complete. Potential bottles detected: {bottle_count}.")
    
        # Return the count of detected potential bottles
        return bottle_count

        

    def calcul_direction(self):
        image_width = 640 #the stream of the camera is in 640*480p
        fov=62.2 # the camera as an a vision angle of 62.2 degres
        if self.result is None:
             logger.info("Erreur: Aucune detection, impossible de calculer la direction.")
            return
        beer_center_x = self.result[0]

        # Calculate the image's center x-coordinate
        image_center_x = image_width / 2

        # Calculate the pixel offset from the center of the image
        pixel_offset = beer_center_x - image_center_x

        # Calculate the degrees per pixel
        degrees_per_pixel = fov / image_width

        # Calculate the angle to turn
        self.direction = pixel_offset * degrees_per_pixel

    def one_image_processing_autonom(self):
        """
        Captures an image from the RTSP stream 
        Compares the current image with the previous one and performs actions based on the difference:
        - <10%: Does nothing.
        - 10%-70%: Processes the image with test_bottle_detectiona and model
        Return the turn angle if a beer is found
        """
        
        # If there is a previous image, compare it with the current image
        if new_image is not None:
            difference = VisionPi.compare_images(self.new_image, self.old_image)

            if difference < 0.1 :
                 logger.info(f"[{datetime.datetime.now()}] Images are too similar (<10%). Skipping.")
            elif 0.1 <= difference :
                 logger.info(f"[{datetime.datetime.now()}] Images are moderately different (10%-70%). Processing.")
                bottle_count=self.test_bottle_detection()
                if bottle_count :
                    self.result=self.run_yolov8nano_model()  
                
    def one_image_processing_hybrid(self):
        """
        Captures an image from the RTSP stream 
        Compares the current image with the previous one and performs actions based on the difference:
        - <10%: Does nothing.
        - 10%-70%: wait for the coral to send a result
        Return the turn angle if a beer is found
        """

        # If there is a previous image, compare it with the current image
        if new_image is not None:
            actual_image = cv2.imread(self.old_image)
            difference = VisionPi.compare_images(self.new_image, self.old_image)

            if difference < 0.1 :
                 logger.info(f"[{datetime.datetime.now()}] Images are too similar (<10%). Skipping.")
            elif 0.1 <= difference :
                 logger.info(f"[{datetime.datetime.now()}] Images are moderately different (10%-70%). waiting for coral.")
                pre_result=self.listener.get_latest_detections()
                #self.result=[pre_result]
                
                
                

        # Update the previous image
        cv2.imwrite(self.old_image, frame)
        self.old_image = cv2.imread(self.new_image)
         logger.info(f"[{datetime.datetime.now()}] Updated previous frame as {self.path1}")
        
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
            self.listener._run()
            while self.running:
                if self.mode == 0: # Pi only
                    new_image=self.mjpeg_server.get_frame()
                     logger.info("Mode: Pi only")
                     logger.info(f" Starting one image processing cycle...")
                    # Process a single cycle
                    self.one_image_processing_autonome()
                    # React based on the state                  
                elif self.mode == 1: # TODO Hybrid mode
                    # self.process_inference_data(data)
                    self.one_image_processing_hybrid()
                    continue
                elif self.mode == 2: # TODO coral mode
                    # self.process_inference_data(data)
                    continue
                else: 
                     print("Unexpected mode: ", self.mode)
                    break
                if self.state == 0:
                     logger.info(f"[{datetime.datetime.now()}] State 0: No action required.")
                        #wait until the coral get the 
                elif self.state == 1:
                     print(f"[{datetime.datetime.now()}] Alert: Bottle detected. Direction = {self.direction:.2f}°")
                    self.message_queue.put({"type": "bottle_detected", "direction": self.direction   })
                # Wait for 4 seconds before repeating
                time.sleep(4)
            self.listener.stop()
        except Exception as e:
             logger.info(f"VisionPi Error: {e}")

if __name__ == "__main__":
  logger.info("hello")



