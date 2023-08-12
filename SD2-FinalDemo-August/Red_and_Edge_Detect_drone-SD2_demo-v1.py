from pickle import STOP
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import logging
import drone_client as connect # make sure drone_client.py file is in the same directory as where this program is run
import Edge_Server_VideoTopic # make sure Edge_Server_VideoTopic.py file is in the same directory as where this program is run
import threading


# Global variables:
SEND_STOP = 0
SEND_CONT_DRIVE = 1
SEND_REDUCE_SPEED = 2
SEND_TURN_LEFT = 3
SEND_TURN_RIGHT = 4
SEND_ALL_CLEAR = 5
SHAPE_APPEARANCE_THRESHOLD = 1
NUM_TRIANGLES = 0
NUM_SQUARES = 0
NUM_HEXAGONS = 0
NUM_CIRCLES = 0
NUM_STARS = 0
RED_OBJ_FOUND = False
SUBSCRIBER_TOPIC = "main_camera/image_raw"
PUBLISHER_TOPIC = 'red_and_edge_object_detection/image_raw'
MESSAGE_CAR_THREAD_RUNNING = threading.Event() 
LAST_COMMAND_SENT = None


# Create a logger instance for file logging
file_logger = logging.getLogger(__name__ + '--file_logger')
file_logger.setLevel(logging.INFO)
file_formatter = logging.Formatter("%(asctime)s - [%(name)s] - %(levelname)s - %(message)s")
file_handler = logging.FileHandler("red_and_edge_object_detection_log.txt", mode="a") #open in append mode so file is not reset
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

def initialize_ros_node():
    try:
        rospy.init_node('red_and_edge_object_detection')
        file_logger.info("ROS node 'red_and_edge_object_detection' initialized.")
        print("ROS node 'red_and_edge_object_detection' initialized.")  
        return True
    except rospy.ROSInitException as e:
        file_logger.error("Failed to initialize red_and_edge_object_detection ROS node: %s", str(e))
        console_logger.error("Failed to initialize red_and_edge_object_detection ROS node: %s", str(e))  
        return False

def send_message_to_car_thread():
    
    while True:
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


def detect_red(cv_image):
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    lower_red1 = (0, 100, 20)
    upper_red1 = (10, 255, 255)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

    lower_red2 = (160, 100, 20)
    upper_red2 = (179, 255, 255)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    mask = cv2.bitwise_or(mask1, mask2)

    return mask

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
    global RED_OBJ_FOUND

    RED_OBJ_FOUND = False  # Initialize RED_OBJ_FOUND flag to False

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Find contours in the mask image using RETR_EXTERNAL mode and CHAIN_APPROX_SIMPLE method

    for contour in contours:
        area = cv2.contourArea(contour)  # Calculate the area of the contour

        # Check if the contour area is within the specified range
        if min_area < area < max_area: 
            # Set RED_OBJ_FOUND flag to True if at least one contour meets the criteria
            RED_OBJ_FOUND = True  

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

def image_callback(data):
    global RED_OBJ_FOUND, LAST_COMMAND_SENT
    bridge = CvBridge()

    try:
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')

        mask = detect_red(cv_image)

        cv_image_with_bboxes = draw_bounding_boxes(cv_image, mask)

        # Logic used to determine what message needs to be sent to the car (this can be easily moved closer to when red object was found, if desired):
            # For each case, we will make sure that the send_message thread is not already running in order to reduce spamming.

            # send a stop if a red object found and stop was not last message sent (prevents sending consecutive stop commands)
        if RED_OBJ_FOUND and (LAST_COMMAND_SENT != SEND_STOP) and not MESSAGE_CAR_THREAD_RUNNING.is_set():
            LAST_COMMAND_SENT = SEND_STOP
            MESSAGE_CAR_THREAD_RUNNING.set() # trigger send_message to car event thread to send stop command to car
                
            # continue --> (prevents sneding consecutive stop commands)
        elif RED_OBJ_FOUND and (LAST_COMMAND_SENT == SEND_STOP):
            pass
            
            # command car to recover and continue driving if no object found and last command it received was a STOP command
        elif not RED_OBJ_FOUND and (LAST_COMMAND_SENT == SEND_STOP) and not MESSAGE_CAR_THREAD_RUNNING.is_set():
            LAST_COMMAND_SENT = SEND_CONT_DRIVE
            MESSAGE_CAR_THREAD_RUNNING.set() # trigger send_message to car event thread to send cont drive command to car
                
            #otherwise, inform car the all clear (use addtional if statement to prevent sending consecutive all clear commands)
        else:
            if LAST_COMMAND_SENT != SEND_ALL_CLEAR and not MESSAGE_CAR_THREAD_RUNNING.is_set():
                LAST_COMMAND_SENT = SEND_ALL_CLEAR
                MESSAGE_CAR_THREAD_RUNNING.set() # trigger send_message to car event thread to send all clear to car
            else:
                pass
           
        # Convert image back to format for posting to an image topic: 
        img_msg = bridge.cv2_to_imgmsg(cv_image_with_bboxes, encoding="bgr8")

        # Send image to edge server video topic so that it can be obtained and processed by the car edge server
        # if no red object found
        if not RED_OBJ_FOUND:
            Edge_Server_VideoTopic.EdgeServer_image_pub.publish(img_msg)

        # Still publish image to red and edge video topic whether red object found or not, just to have a continuous stream
        # that can be used for testing/viewing if our program is working as expected:
        red_and_edge_image_pub.publish(img_msg)

    except Exception as e:
        file_logger.error("Error in image_callback: %s", str(e))
        console_logger.error("Error in image_callback: %s", str(e))
        

def start_image_processing(subscriber_topic):
    try:
        image_sub = rospy.Subscriber(subscriber_topic, Image, image_callback)
        rospy.spin()

    except rospy.ROSException as e:
        file_logger.error("ROSException in start_image_processing: %s", str(e))
        console_logger.error("ROSException in start_image_processing: %s", str(e))
        

def main():
    
    # Start ros node and connect to car command server:
    if not initialize_ros_node(): # initialize ros node for posting topic
        return
    if not connect_to_car_command_server(): # connect to car command server
        return
    
    # Start thread that will wait to be awakened to send a message to the car:
    threading.Thread(target=send_message_to_car_thread).start()

    global red_and_edge_image_pub
    red_and_edge_image_pub = rospy.Publisher(PUBLISHER_TOPIC, Image, queue_size=10)
    file_logger.info("ROS publisher created for %s topic.", PUBLISHER_TOPIC)
    print("ROS publisher created for %s topic." % PUBLISHER_TOPIC)

    start_image_processing(SUBSCRIBER_TOPIC)

if __name__ == "__main__":
    main()

#Special Note on rospy.init and rospy.spin functions:
"""
# Initialization: When you create a ROS node using rospy.init_node('node_name'), you set up a node with a specific name.

# Subscriptions, Publications, and Services: Within this node, you can define various activities,
# such as subscribing to topics, publishing messages, and providing or using services.

# Event Loop: When you call rospy.spin(), you essentially start the event loop of the ROS node.
# This loop continuously checks for incoming events like new messages on subscribed topics or service calls.

# Event Handling: Whenever an event is received, such as a new message on a subscribed topic,
# the corresponding callback function associated with that event is executed.

# Continual Operation: The rospy.spin() function does not exit on its own.
# It keeps the node alive and responsive to events.
# This is essential for maintaining communication and interaction with other nodes in a ROS system.
"""