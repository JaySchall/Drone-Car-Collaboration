import subprocess
import datetime

def run_ros_command(command):
    try:
        subprocess.run(command, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error while running ROS command: {e}")

def main():
    image_topic = "/main_camera/image_raw"
    save_path = input("Enter the save path for the video: ")

    # Generate a timestamp for the current date and time
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    # Append the timestamp to the save path
    save_file = f"{save_path}/out_{timestamp}.avi"

    command = f"rosrun image_view video_recorder image:={image_topic} _filename:={save_file}"
    run_ros_command(command)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nRecording stopped by user.")
