# This module is to be run as a main module on a node with darknet (yolov4) installed

import cv2
import numpy as np
from darknet import *
import logging
import car_edge_server as connect

# Global variables
SEND_STOP = 0  # Stop command
SEND_CONT_DRIVE = 1  # Continue driving command
SEND_REDUCE_SPEED = 2  # Reduce speed command
SEND_TURN_LEFT = 3  # Turn left command
SEND_TURN_RIGHT = 4  # Turn right command
SEND_ALL_CLEAR = 5  # All clear command

# Create a logger instance
yoloShapeDetect_logger = logging.getLogger(__name__)

# Configure logging to write to a log file and console
yoloShapeDetect_logger.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s - [%(name)s] - %(levelname)s - %(message)s")
file_handler = logging.FileHandler("yoloShapeDetect_log.txt", mode="a") #open in append mode so log file is not reset
file_handler.setFormatter(formatter)
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
yoloShapeDetect_logger.addHandler(file_handler)
yoloShapeDetect_logger.addHandler(stream_handler)

def connect_to_car_command_server():
    yoloShapeDetect_logger.info("Trying to establish connection with car command server...")
    if not connect.establish_socket_connection():
        yoloShapeDetect_logger.info("Failed to establish connection with the car command server.")
        return False
    return True

def send_message_to_car(command):
    yoloShapeDetect_logger.info("Sending message %s to car [0=stop, 1=cont_drive, 2=red_speed, 3=L, 4=R, 5=clear]", command)
    connect.message_car(command)

def darknet_helper(img, width, height, network, class_names):

    # darknet helper function to run detection on image
    darknet_image = make_image(width, height, 3)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_resized = cv2.resize(img_rgb, (width, height),
                              interpolation=cv2.INTER_LINEAR)

    # get image ratios to convert bounding boxes to proper size
    img_height, img_width, _ = img.shape
    width_ratio = img_width / width
    height_ratio = img_height / height

    # run model on darknet style image to get detections
    copy_image_from_bytes(darknet_image, img_resized.tobytes())
    detections = detect_image(network, class_names, darknet_image)
    free_image(darknet_image)

    return detections, width_ratio, height_ratio

def main():

    SUBSCRIBER_TOPIC = "/main_camera/image_raw"

    # Connect to car command server
    if not connect_to_car_command_server():
        return

    # Load in our YOLOv4 architecture network
    network, class_names, class_colors = load_network("yolov4-tiny-custom.cfg", "obj.data", "yolov4-tiny-custom_last.weights")
    width = network_width(network)
    height = network_height(network)
    
    # URL of the video stream
    video_url = "http://192.168.11.1:8080/stream?topic=" + SUBSCRIBER_TOPIC

    # Create video capture object:
    cap = cv2.VideoCapture(video_url)

    if not cap.isOpened():
        yoloShapeDetect_logger.error("Failed to open the video stream.")
        return

    # Initialize object detected variable to false
    obj_detected = False

    # Read and display video frames until the user presses 'q'
    while True:
        # Read the next frame from the video stream
        ret, frame = cap.read()

        # If the frame was not successfully read, then we have reached the end of the stream
        if not ret:
            yoloShapeDetect_logger.error("Failed to read video stream; end of stream reached or stream/video error.")
            break

        # Set obj_detected to false here which is used to determine if a command should be sent to stop the car
        obj_detected = False

        # Darknet detection and display
        detections, width_ratio, height_ratio = darknet_helper(frame, width, height, network, class_names)
        for label, confidence, bbox in detections:
            left, top, right, bottom = bbox2points(bbox)
            left, top, right, bottom = int(left * width_ratio), int(top * height_ratio), int(right * width_ratio), int(bottom * height_ratio)
            cv2.rectangle(frame, (left, top), (right, bottom), class_colors[label], 2)
            cv2.putText(frame, "{} [{:.2f}]".format(label, float(confidence)),
                        (left, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        class_colors[label], 2)
            # Set obj_detected to true - for loop entered; at least one object was detected in the frame (detections list != empty)
            obj_detected = True

        # If obj_detected is true, send stop to car; otherwise, send continue driving
        if obj_detected:
            send_message_to_car(SEND_STOP)  # Stop the car
        else:
            send_message_to_car(SEND_ALL_CLEAR)  # Continue driving

        # Increase size of frame displayed
        new_width = 3 * frame.shape[1]  # Triple the original width
        new_height = 3 * frame.shape[0]  # Triple the original height

        # Resize the image
        expanded_image = cv2.resize(frame, (new_width, new_height))

        # Display the frame
        cv2.imshow("Edge Server", expanded_image)

        # Wait for the 'q' key to be pressed to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object and close any open windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
