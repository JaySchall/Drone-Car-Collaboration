# This code is used to start the edge server video topic so that the edge server can subscribe to this topic
# via a web interface and process frames in YOLOv4. This will allow a load balance mechanism
# so that the edge server only receives frames not processed by the drone for object detection.
# This module should be imported by Red_and_Edge_detect program

"""Special Note on rospy.spin()

In ROS (Robot Operating System) for Python (rospy), the rospy.spin() function 
is used to keep your Python program running and to process incoming messages from subscribed topics, services, and actions.

When you call rospy.spin(), it enters a loop that keeps your ROS node alive and responsive to incoming messages. It starts processing 
the messages received from the subscribed topics and calls the corresponding callback functions to handle the data.

Once you call rospy.spin(), any remaining code will not be executed, as processing is handed over to that function indefinitely.

Also note: initialization of ros node is not necessary if a script is imported into a 
python script that already initializes a ROS node for the current process. ROS only allows one ROS node per process.
"""

import rospy
from sensor_msgs.msg import Image
import logging


# Global Variables
SUBSCRIBER_TOPIC = 'main_camera/image_raw'
PUBLISHER_TOPIC = 'EdgeServer_VideoTopic/image_raw'
EdgeServer_image_pub = None

# Clear log file (by opening in write mode) before reopening in append mode
with open("red_and_edge_object_detection_log.txt", mode="w"):
    pass

# Create a logger instance for file logging
file_logger = logging.getLogger(__name__ + '--file_logger')
file_logger.setLevel(logging.INFO)
file_formatter = logging.Formatter("%(asctime)s - [%(name)s] - %(levelname)s - %(message)s")
file_handler = logging.FileHandler("red_and_edge_object_detection_log.txt", mode="a")  # Now open in append mode
file_handler.setFormatter(file_formatter)
file_logger.addHandler(file_handler)

# Create a logger instance for console logging
console_logger = logging.getLogger(__name__ + '--console_logger')
console_logger.setLevel(logging.INFO)  # Adjust console logger level as needed
console_formatter = logging.Formatter("%(asctime)s - [%(name)s] - %(levelname)s - %(message)s")
console_handler = logging.StreamHandler()
console_handler.setFormatter(console_formatter)
console_logger.addHandler(console_handler)

# note, the following initialization of ros node is not necessary if this file is imported into a python script that already initializes
# a ros node for the current process. ROS only allows one ROS node per process.
def initialize_ros_node():
    try:
        rospy.init_node('EdgeServer_VideoTopic')
        file_logger.info("ROS node 'EdgeServer_VideoTopic' initialized.")
        console_logger.info("ROS node 'EdgeServer_VideoTopic' initialized.")  # Console logger output
        return True
    except rospy.ROSInitException as e:
        file_logger.error("Failed to initialize ROS EdgeServer_VideoTopic node: %s", str(e))
        console_logger.error("Failed to initialize ROS EdgeServer_VideoTopic node: %s", str(e))  # Console logger output
        return False

def image_callback(data):
    try:
        EdgeServer_image_pub.publish(data)

    except Exception as e:
        file_logger.error("Error in image_callback: %s", str(e))
        console_logger.error("Error in image_callback: %s", str(e))  # Console logger output

def start_image_subscriber(subscriber_topic):
    try:
        image_sub = rospy.Subscriber(subscriber_topic, Image, image_callback)

    except rospy.ROSException as e:
        file_logger.error("ROSException in start_image_subscriber: %s", str(e))
        console_logger.error("ROSException in start_image_subscriber: %s", str(e))  # Console logger output

def main():
    global EdgeServer_image_pub
    EdgeServer_image_pub = rospy.Publisher(PUBLISHER_TOPIC, Image, queue_size=10)
    file_logger.info("ROS publisher created for %s topic.", PUBLISHER_TOPIC)
    console_logger.info("ROS publisher created for %s topic.", PUBLISHER_TOPIC)  # Console logger output

if __name__ == "__main__":
    file_logger.info("Attempting to initialize ROS node and publish EdgeServer_VideoTopic as the main program (not as an imported program)")
    console_logger.info("Attempting to initialize ROS node and publish EdgeServer_VideoTopic as the main program (not as an imported program)")  # Console logger output
    if not initialize_ros_node():
        pass
    else:
        main()
        start_image_subscriber(SUBSCRIBER_TOPIC)
        file_logger.info("Subscribed to %s topic and processing received frames.", SUBSCRIBER_TOPIC)
        console_logger.info("Subscribed to %s topic and processing received frames.", SUBSCRIBER_TOPIC)  # Console logger output
        rospy.spin()
else:
    file_logger.info("%s module imported...ROS node should already be initiated...Attempting to publish EdgeServer_VideoTopic", __name__)
    console_logger.info("%s module imported...ROS node should already be initiated...Attempting to publish EdgeServer_VideoTopic", __name__)  # Console logger output
    main()

