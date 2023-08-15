import cv2
s = "grouped-sunny-out"
capture = cv2.VideoCapture(f'{s}.avi')
 
frameNr = 0
 
while (True):
 
    success, frame = capture.read()
 
    if success:
        cv2.imwrite(f'{s}-{frameNr}.jpg', frame)
 
    else:
        break
 
    frameNr = frameNr+1
 
capture.release()