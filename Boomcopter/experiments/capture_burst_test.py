#!/usr/bin/python
import io
import time
import threading
import picamera
from PIL import Image
import numpy as np
import cv2

# Create a pool of image processors
done = False
lock = threading.Lock()
pool = []
last_time = 0

class ImageProcessor(threading.Thread):
    def __init__(self):
        super(ImageProcessor, self).__init__()
        self.stream = io.BytesIO()
        self.event = threading.Event()
        self.terminated = False
        self.start()

    def run(self):
        # This method runs in a separate thread
        global done, last_time
        while not self.terminated:
            if self.event.wait(1):
                try:
                    self.stream.seek(0)
                    # Read the image and do some processing on it
                    img = np.asarray(Image.open(self.stream))
                    frame = img

                    # convert the frame to grayscale, blur it, and detect edges
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    blur = cv2.GaussianBlur(gray, (7, 7), 0)
                    edge = cv2.Canny(blur, 50, 150)

                    # find contours in the edge map
                    (_, cnts, _) = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    # loop over the contours
                    for c in cnts:
                    	# approximate the contour
                    	peri = cv2.arcLength(c, True)
                    	approx = cv2.approxPolyDP(c, 0.01 * peri, True)

                    	# ensure that the approximated contour is "roughly" rectangular
                    	if len(approx) >= 4 and len(approx) <= 7:
							# compute the bounding box of the approximated contour and
							# use the bounding box to compute the aspect ratio
							(x, y, w, h) = cv2.boundingRect(approx)
							aspectRatio = w / float(h)

							# compute the solidity of the original contour
							area = cv2.contourArea(c)
							hullArea = cv2.contourArea(cv2.convexHull(c))
							solidity = area / float(hullArea)

							# compute whether or not the width and height, solidity, and
							# aspect ratio of the contour falls within appropriate bounds
							keepDims = w > 10 and h > 10
							keepSolidity = solidity > 0.9
								# keepSolidity = True
							keepAspectRatio = aspectRatio >= 0.8 and aspectRatio <= 1.2

							# ensure that the contour passes all our tests
							if keepDims and keepSolidity and keepAspectRatio:
								# draw an outline around the target and update the status
								# text
								cv2.drawContours(frame, [approx], -1, (255, 0, 0), 4)

								try:
									# compute the center of the contour region and draw the
									# crosshairs
									M = cv2.moments(approx)
									(cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
									(startX, endX) = (int(cX - (w * 0.15)), int(cX + (w * 0.15)))
									(startY, endY) = (int(cY - (h * 0.15)), int(cY + (h * 0.15)))
									cv2.line(frame, (startX, cY), (endX, cY), (0, 0, 255), 3)
									cv2.line(frame, (cX, startY), (cX, endY), (0, 0, 255), 3)
								except Exception:
									print "Divide by zero"

                    # show the frame and record if a key is pressed
                    cv2.imshow("Frame", frame)
                    key = cv2.waitKey(1) & 0xFF
					
                    current_time = int(round(time.time()*1000))
                    print current_time - last_time
                    last_time = current_time

                    # if the 'q' key is pressed, stop the loop
                    if key == ord("q"):
						done = True
						break
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()
                    # Return ourselves to the pool
                    with lock:
                        pool.append(self)

def streams():
    global lock, done, pool
    while not done:
        with lock:
            processor = pool.pop()
        yield processor.stream
        processor.event.set()

with picamera.PiCamera() as camera:
    global lock, done, pool
    pool = [ImageProcessor() for i in range (10)]
    camera.resolution = (640, 480)
    # Set the framerate appropriately; too fast and the image processors
    # will stall the image pipeline and crash the script
    camera.framerate = 10
    # camera.start_preview()
    time.sleep(2)
    camera.capture_sequence(streams(), use_video_port=True)

# Shut down the processors in an orderly fashion
while pool:
    with lock:
        processor = pool.pop()
    processor.terminated = True
    processor.join()

exit()
