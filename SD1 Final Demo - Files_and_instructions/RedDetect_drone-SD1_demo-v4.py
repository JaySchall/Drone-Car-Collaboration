import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import os
import drone_client as connect

#global variables:
SEND_STOP = 0
SEND_CONT_DRIVE = 1
SEND_REDUCE_SPEED = 2
SEND_TURN_LEFT = 3
SEND_TURN_RIGHT = 4
SEND_ALL_CLEAR = 5
RED_OBJ_FOUND = False
#

# When importing drone_client.py,
# make sure it is in the same directory as where
# this program was run from.

# Make sure drone_client.py file is in the same directory as where this program is run:
current_directory = os.getcwd()
sys.path.insert(0, current_directory)
print("Current working directory (where this program was ran from):", current_directory)

print("Trying to establish connection with car server...")
if connect.establish_socket_connection() == False:
    print("Exiting program...")
    exit(1)

try:
    rospy.init_node('red_object_detection')
    print("ROS node 'red_object_detection' initialized.")
except rospy.ROSInitException as e:
    print("Failed to initialize ROS node:", str(e))
    exit(1)  # Exit the program with a non-zero exit status to indicate failure



# Create a publisher to publish the video feed to a ROS topic
image_pub = rospy.Publisher('red_object_detection/image_raw', Image, queue_size=10)
print("ROS publisher created for 'red_object_detection/image_raw' topic.")

def send_message_to_car(command):
        print(f"Sending message {command} to car [0=stop,1=cont_drive,2=red_speed,3=L, 4=R,5=clear]")
        connect.message_car(command)

def detect_red(cv_image):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define the range of red color in HSV (red has 2 ranges) --> format is (hue, saturation, value (brightness))
    # lower range: 0-10 for hue for color red
    lower_red = (0, 100, 20)
    upper_red = (10, 255, 255)
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    # upper range: 170-180 for hue for color red
    lower_red = (160, 100, 20)
    upper_red = (179, 255, 255)
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    # Combine the masks to obtain the final mask
    mask = mask1 + mask2

    return mask

# min and max area determine size of detected objects to draw bounding boxes around
def draw_bounding_boxes(cv_image, mask, min_area=1000, max_area=10000):
    global RED_OBJ_FOUND
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterate through the contours
    for contour in contours:
        # Calculate the area of the contour
        area = cv2.contourArea(contour)

        # Filter contours based on the minimum and maximum area -> if we enter this if-statement, 
        # then a red object is considered detected
        if min_area < area < max_area:
            # Calculate the bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)
            # Draw the bounding box on the original image
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            RED_OBJ_FOUND = True
        else:
            RED_OBJ_FOUND = False
    return cv_image

def image_callback(data):
    global RED_OBJ_FOUND
    bridge = CvBridge()  # Create an instance of the CvBridge class to convert between ROS Image messages and OpenCV images

    try:
        # Convert the ROS image message to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')

        # Detect the red objects in the image
        mask = detect_red(cv_image)

        # Draw bounding boxes around the detected red objects
        cv_image_with_bboxes = draw_bounding_boxes(cv_image, mask)
        # Now Send message to the car if object detected
        if RED_OBJ_FOUND:
            send_message_to_car(SEND_STOP)
            connect.wait_timer(5) #wait for some time before processing and sending more commands
            send_message_to_car(SEND_CONT_DRIVE) #now tell car to continue driving
        else:
            send_message_to_car(SEND_ALL_CLEAR)
        # Convert the OpenCV image back to a ROS image message
        img_msg = bridge.cv2_to_imgmsg(cv_image_with_bboxes, encoding="bgr8")

        # Publish the ROS image message to the topic
        image_pub.publish(img_msg)

    except Exception as e:
        print("Error in image_callback:", str(e))

def start_image_processing():
    try:
        # Create a subscriber to receive video frames from the camera
        image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)

        rospy.spin()

    except rospy.ROSException as e:
        print("ROSException in start_image_processing:", str(e))

if __name__ == "__main__":
    start_image_processing()
