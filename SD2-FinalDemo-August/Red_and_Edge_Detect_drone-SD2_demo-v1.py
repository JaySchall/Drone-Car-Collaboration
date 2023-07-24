import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import logging
import drone_client as connect #make sure drone_client.py file is in the same directory as where this program is run

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

# Configure logging to write to a log file and console
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("red_and_edge_object_detection_log.txt"),
        logging.StreamHandler()
    ]
)

def connect_to_car_command_server():
    logging.info("Trying to establish connection with car command server...")
    if not connect.establish_socket_connection():
        logging.info("Failed to establish connection with the car command server.")
        return False
    return True

def initialize_ros_node():
    try:
        rospy.init_node('red_and_edge_object_detection')
        logging.info("ROS node 'red_and_edge_object_detection' initialized.")
        return True
    except rospy.ROSInitException as e:
        logging.error("Failed to initialize red_and_edge_object_detection ROS node: %s", str(e))
        return False

def send_message_to_car(command):
    logging.info("Sending message %s to car [0=stop, 1=cont_drive, 2=red_speed, 3=L, 4=R, 5=clear]", command)
    connect.message_car(command)

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
    global RED_OBJ_FOUND
    bridge = CvBridge()

    try:
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')

        mask = detect_red(cv_image)

        cv_image_with_bboxes = draw_bounding_boxes(cv_image, mask)

        if RED_OBJ_FOUND:
            send_message_to_car(SEND_STOP)
        else:
            send_message_to_car(SEND_ALL_CLEAR)

        img_msg = bridge.cv2_to_imgmsg(cv_image_with_bboxes, encoding="bgr8")

        image_pub.publish(img_msg)

    except Exception as e:
        logging.error("Error in image_callback: %s", str(e))

def start_image_processing():
    try:
        image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
        rospy.spin()

    except rospy.ROSException as e:
        logging.error("ROSException in start_image_processing: %s", str(e))

def main():
    if not connect_to_car_command_server():
        return

    if not initialize_ros_node():
        return

    global image_pub
    image_pub = rospy.Publisher('red_and_edge_object_detection/image_raw', Image, queue_size=10)
    logging.info("ROS publisher created for 'red_and_edge_object_detection/image_raw' topic.")

    start_image_processing()

if __name__ == "__main__":
    main()
