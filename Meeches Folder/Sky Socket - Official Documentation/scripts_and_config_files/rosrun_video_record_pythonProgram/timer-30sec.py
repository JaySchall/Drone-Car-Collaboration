import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import datetime
import time

bridge = CvBridge()
video_writer = None
start_time = None
duration = None

def image_callback(msg):
    global video_writer, start_time, duration

    if video_writer is None:
        # Start recording
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = "/mnt/usb_drive"
        video_file = f"{save_path}/out_{timestamp}.avi"
        video_writer = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc(*'XVID'), 30, (msg.width, msg.height))

        duration = float(30)
        start_time = time.time()

    # Convert ROS Image to OpenCV format
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Write frame to video
    video_writer.write(cv_image)

    # Check if recording duration is reached
    elapsed_time = time.time() - start_time
    remaining_time = max(0, duration - elapsed_time)
    print(f"Recording... Remaining time: {remaining_time:.1f} seconds")

    if elapsed_time >= duration:
        stop_recording()
        rospy.signal_shutdown("Recording completed.")

def stop_recording():
    global video_writer, start_time, duration

    if video_writer is not None:
        video_writer.release()
        video_writer = None
        duration = None
        start_time = None
        print("Recording stopped.")

def main():
    rospy.init_node("video_recorder_node")
    image_topic = "/main_camera/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
