#This code is used to start the edge server video topic so that the edge server can subscribe to this topic
#via web interface and process frames in yolov4 - this will allow load balance mechanism
#so that the edge server only receives frames not processed by the drone for object detection.

import rospy
from sensor_msgs.msg import Image
import logging

# Configure logging to write to a log file and console
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("red_and_edge_object_detection_log.txt"), #default file mode is append mode
        logging.StreamHandler()
    ]
)

def initialize_ros_node():
    try:
        rospy.init_node('EdgeServer_VideoTopic')
        logging.info("ROS node 'EdgeServer_VideoTopic' initialized.")
        return True
    except rospy.ROSInitException as e:
        logging.error("Failed to initialize ROS EdgeServer_VideoTopic node: %s", str(e))
        return False

def image_callback(data):
    try:
        image_pub.publish(data)

    except Exception as e:
        logging.error("Error in image_callback: %s", str(e))


def start_image_subscriber():
    try:
        image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
        rospy.spin()

    except rospy.ROSException as e:
        logging.error("ROSException in start_image_subscriber: %s", str(e))

def main():

    if not initialize_ros_node():
        return

    global image_pub
    image_pub = rospy.Publisher('EdgeServer_VideoTopic/image_raw', Image, queue_size=10)
    logging.info("ROS publisher created for 'EdgeServer_VideoTopic/image_raw' topic.")

    start_image_subscriber()



if __name__ == "__main__":
    logging.info("Attempting to publish EdgeServer_VideoTopic as main program (not as imnported program)")
    main()
else:
    logging.info("%s module imported...Attempting to publish EdgeServer_VideoTopic", __name__)
    main()
