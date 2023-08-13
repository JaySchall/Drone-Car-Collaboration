# This module is to be run as a main module on a node with darknet (yolov4) installed

import cv2
import numpy as np
from darknet import *
import logging
import car_edge_server as connect
import threading

# Global variables
SEND_STOP = 0  # Stop command
SEND_CONT_DRIVE = 1  # Continue driving command
SEND_REDUCE_SPEED = 2  # Reduce speed command
SEND_TURN_LEFT = 3  # Turn left command
SEND_TURN_RIGHT = 4  # Turn right command
SEND_ALL_CLEAR = 5  # All clear command
OBJ_DETECTED = False # track if object was detected
LAST_COMMAND_SENT = None # track last command sent
SUBSCRIBER_TOPIC = "EdgeServer_VideoTopic/image_raw"
 # use these to prevent spamming messages to car, and also to stop the messaging thread:
MESSAGING_THREAD = None
MESSAGE_CAR_THREAD_RUNNING = threading.Event()
LAST_COMMAND_SENT = None
SEND_MESSAGES = True

# Create a logger instance for file logging
file_logger = logging.getLogger(__name__ + '--file_logger')
file_logger.setLevel(logging.INFO)
file_formatter = logging.Formatter("%(asctime)s - [%(name)s] - %(levelname)s - %(message)s")
file_handler = logging.FileHandler("yoloShapeDetect_log.txt", mode="a") #open in append mode so file is not reset
file_handler.setFormatter(file_formatter)
file_logger.addHandler(file_handler)

# Create a logger instance for console logging
console_logger = logging.getLogger(__name__ + '--console_logger')
console_logger.setLevel(logging.INFO)  # Console logger set to capture WARNING and higher messages
console_formatter = logging.Formatter("%(asctime)s - [%(name)s] - %(levelname)s - %(message)s")
console_handler = logging.StreamHandler()
console_handler.setFormatter(console_formatter)
console_logger.addHandler(console_handler)


def connect_to_car_command_server():
    file_logger.warning("Trying to establish connection with car command server...")
    console_logger.warning("Trying to establish connection with car command server...")
    if not connect.establish_socket_connection():
        file_logger.error("Failed to establish connection with the car command server.")
        console_logger.error("Failed to establish connection with the car command server.")
        return False
    return True

def send_message_to_car_thread():
    try:
        while SEND_MESSAGES:
            if not MESSAGE_CAR_THREAD_RUNNING.is_set():
                file_logger.info("SendMessage thread waiting...Standing by to send a message...")
                console_logger.info("SendMessage thread waiting...Standing by to send a message...")
                MESSAGE_CAR_THREAD_RUNNING.wait() # send thread to sleep (spin) --> wait for main thread to wake this thread
        
            if not connect.CONNECTED_TO_SERVER: # make sure connection is active before attempting to send message.
                file_logger.warning("No connection with car command server; need to reestablsh connection...")
                console_logger.warning("No connection with car command server; need to reestablsh connection...")
                if not connect_to_car_command_server(): # try to reestablish connection
                    file_logger.error("Connection reestablishment failed: No connection with car command server...")
                    console_logger.error("Connection reestablishment failed: No connection with car command server...")
            else:
                command = LAST_COMMAND_SENT
                if command == SEND_STOP:
                    file_logger.warning("Sending message %s to car [0=stop, 1=cont_drive, 2=red_speed, 3=L, 4=R, 5=clear]", command)
                    console_logger.warning("Sending message %s to car [0=stop, 1=cont_drive, 2=red_speed, 3=L, 4=R, 5=clear]", command)
                else:
                    file_logger.info("Sending message %s to car [0=stop, 1=cont_drive, 2=red_speed, 3=L, 4=R, 5=clear]", command)
                    console_logger.info("Sending message %s to car [0=stop, 1=cont_drive, 2=red_speed, 3=L, 4=R, 5=clear]", command)
                connect.message_car(command) # send the message

            MESSAGE_CAR_THREAD_RUNNING.clear() # clear flag so main thread knows it can now send another message.
        
        # SEND_MESSAGES == False; thus exiting this thread.
        file_logger.warning("SEND_MESSAGES == FALSE --> Terminating Messaging Thread...")
        console_logger.warning("SEND_MESSAGES == FALSE --> Terminating Messaging Thread...")
    except KeyboardInterrupt:
        print("\nCTRL + C detected from user input. Exiting the program...")
        file_logger.error("\nCTRL + C detected from user input. Exiting the program...", str(e))
        exit_program()
    except Exception as e:
        file_logger.error("Error in image_callback: %s", str(e))
        console_logger.error("Error in image_callback: %s", str(e))
        exit_program()

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

def exit_program():
    global SEND_MESSAGES
    
    # Join connection thread with main thread before exiting (smoother exiting)
    SEND_MESSAGES = False               # set to false so thread will not to process anymore messages
    MESSAGE_CAR_THREAD_RUNNING.set()    # wake thread so it can terminate when it checks send messages boolean
    connect.close_socket()              # close socket
    # Joining all active threads
    for thread in threading.enumerate():
        if thread is not threading.currentThread():
            thread.join()
 

def main():
    global OBJ_DETECTED, MESSAGING_THREAD
    # Connect to car command server
    if not connect_to_car_command_server():
        return

    # Create and start global messaining thread that will wait to be awakened to send a message to the car:
    MESSAGING_THREAD = threading.Thread(target=send_message_to_car_thread)
    MESSAGING_THREAD.start()

    # Load in our YOLOv4 architecture network
    network, class_names, class_colors = load_network("yolov4-tiny-custom.cfg", "obj.data", "yolov4-tiny-custom_last.weights")
    width = network_width(network)
    height = network_height(network)
    
    # URL of the video stream
    video_url = "http://192.168.11.1:8080/stream?topic=" + SUBSCRIBER_TOPIC

    # Create video capture object:
    cap = cv2.VideoCapture(video_url)

    if not cap.isOpened():
        file_logger.error("Failed to open the video stream.")
        console_logger.error("Failed to open the video stream.")
        return

    # Initialize object detected variable to false
    OBJ_DETECTED = False

    # Read and display video frames until the user presses 'q'
    while True:
        # Read the next frame from the video stream
        ret, frame = cap.read()

        # If the frame was not successfully read, then we have reached the end of the stream
        if not ret:
            file_logger.error("Failed to read video stream; end of stream reached or stream/video error.")
            console_logger.error("Failed to read video stream; end of stream reached or stream/video error.")
            continue

        # Set obj_detected to false here which is used to determine if a command should be sent to stop the car
        OBJ_DETECTED = False

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
            OBJ_DETECTED = True

        # Logic used to determine what message needs to be sent to the car (this can be easily moved closer to when red object was found, if desired):
            # For each case, we will make sure that the send_message thread is not already running in order to reduce spamming.

            # send a stop if a red object found and stop was not last message sent (prevents sending consecutive stop commands)
        if OBJ_DETECTED and (LAST_COMMAND_SENT != SEND_STOP) and not MESSAGE_CAR_THREAD_RUNNING.is_set():
            LAST_COMMAND_SENT = SEND_STOP
            MESSAGE_CAR_THREAD_RUNNING.set() # trigger send_message to car event thread to send stop command to car
                
            # continue --> (prevents sneding consecutive stop commands)
        elif OBJ_DETECTED and (LAST_COMMAND_SENT == SEND_STOP):
            pass
            
            # command car to recover and continue driving if no object found and last command it received was a STOP command
        elif not OBJ_DETECTED and (LAST_COMMAND_SENT == SEND_STOP) and not MESSAGE_CAR_THREAD_RUNNING.is_set():
            LAST_COMMAND_SENT = SEND_CONT_DRIVE
            MESSAGE_CAR_THREAD_RUNNING.set() # trigger send_message to car event thread to send cont drive command to car
                
            #otherwise, inform car the all clear (use addtional if statement to prevent sending consecutive all clear commands)
        else:
            if LAST_COMMAND_SENT != SEND_ALL_CLEAR and not MESSAGE_CAR_THREAD_RUNNING.is_set():
                LAST_COMMAND_SENT = SEND_ALL_CLEAR
                MESSAGE_CAR_THREAD_RUNNING.set() # trigger send_message to car event thread to send all clear to car
            else:
                pass
           

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
