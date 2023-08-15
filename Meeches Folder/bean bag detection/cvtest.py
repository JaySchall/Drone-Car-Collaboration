import numpy
import cv2

pos = (300,100)
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 3
color = (250,250,250)
thickness = 5

lowColorVal1 = numpy.array([165, 80, 80])
highColorVal1 = numpy.array([180, 255, 255])
lowColorVal2 = numpy.array([0, 80, 80])
highColorVal2 = numpy.array([10, 255, 255])
#img = cv2.imread("cube.jpg")
cam = cv2.VideoCapture(0)
#if img is None:
#    print("Cannot open image")
#    exit()
if not cam.isOpened():
    print("Cannot open camera")
    exit()

#cv2.imshow("red",img)
#hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#mask = cv2.inRange(hsvimg, lowColorVal, highColorVal)
#cv2.imshow("hsv",hsvimg)
#cv2.imshow("maskred",mask)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
#exit()
cv2.namedWindow('frame',cv2.WINDOW_NORMAL)
cv2.resizeWindow('frame',960,540)
cv2.namedWindow('mask',cv2.WINDOW_NORMAL)
cv2.resizeWindow('mask',960,540)
kernel = numpy.ones((17,17),numpy.uint8)

count = 0

while True:
    label = ""
    # Capture frame-by-frame
    good, frame = cam.read()
    # if frame is read correctly ret is True
    if not good:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    final = frame.copy()
    
    
    mask1 = cv2.inRange(hsv, lowColorVal1, highColorVal1)
    mask2 = cv2.inRange(hsv, lowColorVal2, highColorVal2)
    mask = mask1+mask2
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    maskContours = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]
    
    result = cv2.bitwise_and(frame, frame, mask=mask)
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        num_sides = len(approx)
        if num_sides == 4:
            if count > 3:
                label = "SQUARE"
            count+=1
        elif num_sides == 8:
            if count > 3:
                label = "CIRCLE"
            count+=1
        elif num_sides == 10:
            if count > 3:
                label = "STAR"
            count+=1
        else:
            count = 0
            #print(num_sides)
    # Display the resulting frame
    for contour in maskContours:
        x,y,w,h = cv2.boundingRect(contour)
        cv2.rectangle(final, (x,y), (x + w, y + h), (20, 255, 20), 3)
        cv2.putText(final, label, pos, font, fontScale, color, thickness, cv2.LINE_AA)
    cv2.imshow('frame', final)
    cv2.imshow('mask', blur)
    if cv2.waitKey(1) == ord('q'):
        break
# When everything done, release the captureq
cam.release()
cv2.destroyAllWindows()