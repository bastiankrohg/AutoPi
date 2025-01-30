from asyncio.windows_events import NULL
import signal
import sys
from typing import Any
import rover
import datetime
import time
import tensorflow.lite as tflite
import threading
import queue
import cv2



class VisionCoral:
    def __init__(self,modelpath):
        self.modelpath = modelpath
        self.beer = 0
        self.direction = 0.00
        self.message_queue = queue.message_queue
        self.image= NULL  #image: np.ndarray, Input image to be processed
        self.image_width = 640  #the stream of the camera is in 640*480p
        self.interpreter==NULL
    
    ## functions for the Tensorflow model preparation and utilisation   
    def load_tflite_model(self):
        """
        Load the TensorFlow Lite model.
        """
        print(f"[{datetime.now()}] Loading TensorFlow Lite model from {self.modelpath}...")
        interpreter = tflite.Interpreter(model_path=self.modelpath)
        interpreter.allocate_tensors()
        return interpreter

    def run_tflite_model(self, interpreter, input_image):
        """
        Run inference with the TFLite model on an input image.
        """
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
       
        boxes = interpreter.get_tensor(output_details[0]['index'])  # Bounding boxes
        scores = interpreter.get_tensor(output_details[2]['index'])  # Confidence scores

        # Iterate through detections
        for i in range(len(scores)):
            if scores[i] > 0.5:  # High confidence
                # Extract bounding box and calculate x-center
                ymin, xmin, ymax, xmax = boxes[i]
                if image_size:  # Scale coordinates back to original image size
                    xmin = int(xmin * self.image_width)
                    xmax = int(xmax * self.image_width)
                x_center = (xmin + xmax) / 2.0
                return scores[i], x_center  # Return confidence and x-center

        # No detection for the target class
        return None, None

       
    def process_image(self):
        """
        Process cropped images using a TensorFlow Lite model to detect bottles.
        Stops as soon as a bottle is detected and returns its position.
        """
        
        if self.interpreter==NULL : self.interpreter = self.load_tflite_model(self.modelpath)
        
        # Run recognition using the model
        prediction = self.run_tflite_model(self.interpreter, self.image)

        # Assume the model returns probabilities for each class (e.g., [prob_non_beer, prob_beer])
        is_beer = prediction[0] > 0.5  # If the probability of "beer" is greater than 0.5
        position = prediction[1]

        if is_beer:
            self.beer=1
            return position  # Return immediately when a beer is detected

        
        self.beer=0
        return None  # Return None if no beer is found
    
    def calcul_direction(self,position):
        fov=62.2 # the camera as an a vision angle of 62.2 degres
        x, _, w, _ = position
        beer_center_x = x + (w / 2)

        # Calculate the image's center x-coordinate
        image_center_x = self.image_width / 2

        # Calculate the pixel offset from the center of the image
        pixel_offset = beer_center_x - image_center_x

        # Calculate the degrees per pixel
        degrees_per_pixel = fov / self.image_width

        # Calculate the angle to turn
        self.direction = pixel_offset * degrees_per_pixel


    def run(self):
            """
            Runs the image processing loop every 2 seconds.
            
            """

            while True:
                print(f"[{datetime.now()}] Starting one image processing cycle...")
            
                #wait for an image
                
                # when an image is recieved
                position=self.process_image()
                if self.beer == 1:
                    self.direction=self.calcul_direction(position)
                    print(f" Alert: Bottle detected. Direction = {self.direction:.2f}�")
                    #send beer_detected and direction to pi
                elif self.beer == 0:
                    print(f" Image processed, No beer found")
                self.beer=0

    

if __name__ == "__main__":
    NULL



