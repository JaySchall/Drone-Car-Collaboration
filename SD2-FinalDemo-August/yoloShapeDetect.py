import cv2
import numpy as np
from darknet import *
import logging
import car_edge_server as connect #make sure car_edge_server.py file is in the same directory as where this program is run

# Configure logging to write to a log file and console
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("yoloShapeDetect_log.txt"),
        logging.StreamHandler()
    ]
)

#########
#car command server connection code and definitions begin here:
#########

def connect_to_car_command_server():
    logging.info("Trying to establish connection with car command server...")
    if not connect.establish_socket_connection():
        logging.info("Failed to establish connection with the car command server.")
        return False
    return True

def send_message_to_car(command):
    logging.info("Sending message %s to car [0=stop, 1=cont_drive, 2=red_speed, 3=L, 4=R, 5=clear]", command)
    connect.message_car(command)


#########
#yolov4 code and definitions begin here:
#########

# darknet helper function to run detection on image
def darknet_helper(img, width, height):
  darknet_image = make_image(width, height, 3)
  img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  img_resized = cv2.resize(img_rgb, (width, height),
                              interpolation=cv2.INTER_LINEAR)

  # get image ratios to convert bounding boxes to proper size
  img_height, img_width, _ = img.shape
  width_ratio = img_width/width
  height_ratio = img_height/height

  # run model on darknet style image to get detections
  copy_image_from_bytes(darknet_image, img_resized.tobytes())
  detections = detect_image(network, class_names, darknet_image)
  free_image(darknet_image)
  return detections, width_ratio, height_ratio

# URL of the video stream
# load in our YOLOv4 architecture network
network, class_names, class_colors = load_network("yolov4-tiny-custom.cfg", "obj.data", "yolov4-tiny-custom_last.weights")
width = network_width(network)
height = network_height(network)
video_url = "http://192.168.11.1:8080/stream?topic=/main_camera/image_raw" #http://192.168.11.1:8080/stream?topic=/main_camera/image_raw
net = cv2.dnn.readNetFromDarknet('yolov4-tiny-custom.cfg', 'yolov4-tiny-custom_last.weights')
# Create a VideoCapture object
cap = cv2.VideoCapture(video_url)

# Check if the video capture object was successfully initialized
if not cap.isOpened():
    print("Failed to open the video stream.")
    exit()
out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (320,240))
# Read and display video frames until the user presses 'q'
while True:
    # Read the next frame from the video stream
    ret, frame = cap.read()
    #blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)
    #net.setInput(blob)
    #layer_names = net.getLayerNames()
    #output_layers = [layer_names[i[0][0] - 1] for i in net.getUnconnectedOutLayers()]
    #outputs = net.forward(output_layers)
    # If the frame was not successfully read, then we have reached the end of the stream
    if not ret:
        break

    #obj_detected set false here

    #darknet detection and display
    detections, width_ratio, height_ratio = darknet_helper(frame, width, height)
    for label, confidence, bbox in detections:
        left, top, right, bottom = bbox2points(bbox)
        left, top, right, bottom = int(left * width_ratio), int(top * height_ratio), int(right * width_ratio), int(bottom * height_ratio)
        cv2.rectangle(frame, (left, top), (right, bottom), class_colors[label], 2)
        cv2.putText(frame, "{} [{:.2f}]".format(label, float(confidence)),
                    (left, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    class_colors[label], 2)
        #set obj_detected to true
        

    #if obj_detected true, send stop to car
    #else, send continue

    #increase size of frame displayed
    new_width = 3 * frame.shape[1]  # Double the original width
    new_height = 3 * frame.shape[0] # Double the original height

# Resize the image
    expanded_image = cv2.resize(frame, (new_width, new_height))
    
    # Display the frame
    #cv2.imshow("Video Stream", frame)
    cv2.imshow("Edge Server", expanded_image)
    out.write(frame)
    # Wait for the 'q' key to be pressed to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close any open windows
cap.release()
out.release()
cv2.destroyAllWindows()