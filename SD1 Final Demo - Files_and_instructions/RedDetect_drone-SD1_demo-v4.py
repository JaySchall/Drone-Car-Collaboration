import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import os

# Make sure drone_client.py file is in the same directory as where this program is run:
current_directory = os.getcwd()
sys.path.insert(0, current_directory)

# Here, importing drone_client.py, which should be in the same directory as where this program was run from (as mentioned above):
import drone_client as connect

rospy.init_node('red_object_detection')
bridge = CvBridge()
USED = 0
# Create a publisher to publish the video feed to a ROS topic
image_pub = rospy.Publisher('red_object_detection/image_raw', Image, queue_size=10)

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

#min and max area determine size of detected objects to draw bounding boxes around
def draw_bounding_boxes(cv_image, mask, min_area=1000, max_area=10000):
    # Find contours in the mask
    global USED
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterate through the contours
    for contour in contours:
        # Calculate the area of the contour
        area = cv2.contourArea(contour)

        # Filter contours based on the minimum and maximum area
        if min_area < area < max_area:
            # Calculate the bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)
            if USED < 1:
                USED = USED + 1
                print("Sending message to car...")
                connect.send_message(1)
            # Draw the bounding box on the original image
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return cv_image

def image_callback(data):
    # Convert the ROS image message to an OpenCV image
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')

    # Detect the red objects in the image
    mask = detect_red(cv_image)

    # Draw bounding boxes around the detected red objects
    cv_image_with_bboxes = draw_bounding_boxes(cv_image, mask)

    # Convert the OpenCV image back to a ROS image message
    img_msg = bridge.cv2_to_imgmsg(cv_image_with_bboxes, encoding="bgr8")

    # Publish the ROS image message to the topic
    image_pub.publish(img_msg)

# Create a subscriber to receive video frames from the camera
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)

rospy.spin()
