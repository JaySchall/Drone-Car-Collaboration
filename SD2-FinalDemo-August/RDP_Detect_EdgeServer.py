# This module is to be run as a main module on a node with darknet (yolov4) installed

import cv2
import numpy as np
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

#Variables for RDP algorithm
SHAPE_APPEARANCE_THRESHOLD = 1
NUM_TRIANGLES = 0
NUM_SQUARES = 0
NUM_HEXAGONS = 0
NUM_CIRCLES = 0
NUM_STARS = 0

OBJ_FOUND = False # track if object was detected
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
file_handler = logging.FileHandler("RDP_Detect_EdgeServer_log.txt", mode="a") #open in append mode so file is not reset
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
 

def detect_red_yellow_blue(cv_image):
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define HSV ranges for red color (red has two sets of ranges)
    lower_red1 = (0, 100, 20)
    upper_red1 = (10, 255, 255)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)

    lower_red2 = (160, 100, 20)
    upper_red2 = (179, 255, 255)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)

    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Define HSV ranges for yellow color
    lower_yellow = (20, 100, 100)
    upper_yellow = (40, 255, 255)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Define HSV ranges for blue color
    lower_blue = (90, 100, 100)
    upper_blue = (120, 255, 255)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Combine the masks for red, yellow, and blue using bitwise OR
    combined_mask = cv2.bitwise_or(mask_red, cv2.bitwise_or(mask_yellow, mask_blue))

    return combined_mask



def determine_shape(num_sides):
    """
    Determines the shape label based on the number of sides of the object.
    Updates the respective shape count global variables.
    The num_sides passed into this function must at least be equal to one of the shapes defined below,
    and that shape should have occurred at least more than once in order to obtain a shape label.
    Otherwise, reset counter for all shapes to 0 since num_sides was not equal to one of the defined shapes.
    """

    global NUM_TRIANGLES, NUM_SQUARES, NUM_HEXAGONS, NUM_CIRCLES, NUM_STARS, SHAPE_APPEARANCE_THRESHOLD

    label = "???"

    if num_sides == 3:
        if NUM_TRIANGLES > SHAPE_APPEARANCE_THRESHOLD:
            label = "TRIANGLE"
        NUM_TRIANGLES += 1
    elif num_sides == 4:
        if NUM_SQUARES > SHAPE_APPEARANCE_THRESHOLD:
            label = "SQUARE"
        NUM_SQUARES += 1
    elif num_sides == 6:
        if NUM_HEXAGONS > SHAPE_APPEARANCE_THRESHOLD:
            label = "HEXAGON"
        NUM_HEXAGONS += 1
    elif num_sides == 8:
        if NUM_CIRCLES > SHAPE_APPEARANCE_THRESHOLD:
            label = "CIRCLE"
        NUM_CIRCLES += 1
    elif num_sides == 10:
        if NUM_STARS > SHAPE_APPEARANCE_THRESHOLD:
            label = "STAR"
        NUM_STARS += 1
    else:
        NUM_TRIANGLES = 0
        NUM_SQUARES = 0
        NUM_HEXAGONS = 0
        NUM_CIRCLES = 0
        NUM_STARS = 0

    return label

def draw_bounding_boxes(cv_image, mask, min_area=1000, max_area=10000):
    global OBJ_FOUND

    OBJ_FOUND = False  # Initialize OBJ_FOUND flag to False

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Find contours in the mask image using RETR_EXTERNAL mode and CHAIN_APPROX_SIMPLE method

    for contour in contours:
        area = cv2.contourArea(contour)  # Calculate the area of the contour

        # Check if the contour area is within the specified range
        if min_area < area < max_area: 
            # Set OBJ_FOUND flag to True if at least one contour meets the criteria
            OBJ_FOUND = True  

            # Calculate the bounding rectangle coordinates
            x, y, w, h = cv2.boundingRect(contour) 

            # Draw a green rectangle on the original image using the bounding rectangle coordinates
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Approximate the contour to determine the number of sides using the Ramer-Douglas-Peucker algorithm;
            # the LARGER the Epsilon value,
            # the LESS accurate the number of points connecting the arc will be,
            # since the number of points that make up the arc will be more reduced - less line granularity (resolution)):
            #For more info, see this video: https://www.youtube.com/watch?v=M0J_yq49Go8
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            num_sides = len(approx)
            
            # Determine the shape label for the red object
            shape_label = determine_shape(num_sides)
            
            # Add the shape label to the image
            cv2.putText(cv_image, shape_label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    return cv_image  # Return the modified image with bounding boxes






def main():
    global OBJ_FOUND, MESSAGING_THREAD, LAST_COMMAND_SENT
    # Connect to car command server
    if not connect_to_car_command_server():
        return

    # Create and start global messaining thread that will wait to be awakened to send a message to the car:
    MESSAGING_THREAD = threading.Thread(target=send_message_to_car_thread)
    MESSAGING_THREAD.start()
    
    # URL of the video stream
    video_url = "http://192.168.11.1:8080/stream?topic=" + SUBSCRIBER_TOPIC

    # Create video capture object:
    cap = cv2.VideoCapture(video_url)

    if not cap.isOpened():
        file_logger.error("Failed to open the video stream.")
        console_logger.error("Failed to open the video stream.")
        return

    # Initialize object found variable to false
    OBJ_FOUND = False

    # Read and display video frames until the user presses 'q'
    while True:
        # Read the next frame from the video stream
        cv_image_returned, cv_image = cap.read()

        # If the frame was not successfully read, then we have reached the end of the stream
        if not cv_image_returned:
            file_logger.error("Failed to read video stream; end of stream reached or stream/video error.")
            console_logger.error("Failed to read video stream; end of stream reached or stream/video error.")
            continue

        # Set obj_found to false here which is used to determine if a command should be sent to stop the car
        OBJ_FOUND = False
        
        # Do object detection here:
        mask = detect_red_yellow_blue(cv_image)
        cv_image_with_bboxes = draw_bounding_boxes(cv_image, mask)

        # Logic used to determine what message needs to be sent to the car (this can be easily moved closer to when red object was found, if desired):
            # For each case, we will make sure that the send_message thread is not already running in order to reduce spamming.

            # send a stop if a object found and stop was not last message sent (prevents sending consecutive stop commands)
        if OBJ_FOUND and (LAST_COMMAND_SENT != SEND_STOP) and not MESSAGE_CAR_THREAD_RUNNING.is_set():
            LAST_COMMAND_SENT = SEND_STOP
            MESSAGE_CAR_THREAD_RUNNING.set() # trigger send_message to car event thread to send stop command to car
                
            # continue --> (prevents sending consecutive stop commands)
        elif OBJ_FOUND and (LAST_COMMAND_SENT == SEND_STOP):
            pass
            
            # command car to recover and continue driving if no object found and last command it received was a STOP command
        elif not OBJ_FOUND and (LAST_COMMAND_SENT == SEND_STOP) and not MESSAGE_CAR_THREAD_RUNNING.is_set():
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
        new_width = 3 * cv_image.shape[1]  # Triple the original width
        new_height = 3 * cv_image.shape[0]  # Triple the original height

        # Resize the image
        expanded_image = cv2.resize(cv_image, (new_width, new_height))

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
