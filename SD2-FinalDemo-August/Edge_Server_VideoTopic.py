# This code is used to start the edge server video topic so that the edge server can subscribe to this topic
# via a web interface and process frames in YOLOv4. This will allow a load balance mechanism
# so that the edge server only receives frames not processed by the drone for object detection.
# This module should be imported by Red_and_Edge_detect program

import rospy
from sensor_msgs.msg import Image
import logging


# Clear log file (by opening in write mode) before reopening in append mode
with open("red_and_edge_object_detection_log.txt", mode="w"):
    pass

# Create a logger instance
EdgeServer_video_logger = logging.getLogger(__name__)

# Configure logging to write to a log file and console
EdgeServer_video_logger.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s - [%(name)s] - %(levelname)s - %(message)s")
file_handler = logging.FileHandler("red_and_edge_object_detection_log.txt", mode="a") # now open in append mode
file_handler.setFormatter(formatter)
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
EdgeServer_video_logger.addHandler(file_handler)
EdgeServer_video_logger.addHandler(stream_handler)

# note, the following initialization of ros node is not necessary if this file is imported into a python script that already initializes
# a ros node for the current process. ROS only allows one ROS node per process.
def initialize_ros_node():
    try:
        rospy.init_node('EdgeServer_VideoTopic')  # Initialize the ROS node with the name 'EdgeServer_VideoTopic'
        EdgeServer_video_logger.info("ROS node 'EdgeServer_VideoTopic' initialized.")
        return True
    except rospy.ROSInitException as e:
        EdgeServer_video_logger.error("Failed to initialize ROS EdgeServer_VideoTopic node: %s", str(e))
        return False

def image_callback(data):
    try:
        EdgeServer_image_pub.publish(data)  # Publish the received image data to the 'EdgeServer_VideoTopic/image_raw' topic

    except Exception as e:
        EdgeServer_video_logger.error("Error in image_callback: %s", str(e))

def start_image_subscriber(subscriber_topic):
    try:
        image_sub = rospy.Subscriber(subscriber_topic, Image, image_callback)  # Subscribe to 'main_camera/image_raw' topic
        #rospy.spin()  # uncomment this line to continuously subscribe to new data uploaded by subscriber topic and process using image_callback

    except rospy.ROSException as e:
        EdgeServer_video_logger.error("ROSException in start_image_subscriber: %s", str(e))

def main():

    SUBSCRIBER_TOPIC = 'main_camera/image_raw'
    PUBLISHER_TOPIC = 'EdgeServer_VideoTopic/image_raw'

    global EdgeServer_image_pub
    EdgeServer_image_pub = rospy.Publisher(PUBLISHER_TOPIC, Image, queue_size=10)  # Create a publisher for the 'EdgeServer_VideoTopic/image_raw' topic
    EdgeServer_video_logger.info("ROS publisher created for %s topic.", PUBLISHER_TOPIC)

    start_image_subscriber(SUBSCRIBER_TOPIC)  # Start the image subscriber to process received frames

if __name__ == "__main__":
    EdgeServer_video_logger.info("Attempting to initialize ROS node and publish EdgeServer_VideoTopic as the main program (not as an imported program)")
    if not initialize_ros_node():
        # Note, the following initialization of ros node is not necessary if this file is imported into a 
        # python script that already initializes a ROS node for the current process. ROS only allows one ROS node per process.
        pass
    else:
        main()  # Run the main function if this script is executed directly
else:
    EdgeServer_video_logger.info("%s module imported...Attempting to publish EdgeServer_VideoTopic", __name__)
    # Note: since the script was imported into a script that already initialized the ROS node, no need to init ROS node.

    main()  # still run the main function if this script is imported as a module
