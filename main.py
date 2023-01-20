# Python program to illustrate the concept
# of threading
import threading
import os

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note
import cv2
import time
from collections import deque
import imutils
from imutils.video import VideoStream
import numpy as np
import argparse
import time
import cv2  # opencv - display the video stream
import depthai  # depthai - access the camera and its data packets



robot = Create3(Bluetooth())


pipeline = depthai.Pipeline()

cam_rgb = pipeline.create(depthai.node.ColorCamera)
cam_rgb.setPreviewSize(300, 300)
cam_rgb.setInterleaved(False)

xout_rgb = pipeline.create(depthai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

device = depthai.Device(pipeline)
q_rgb = device.getOutputQueue("rgb")

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
                help="D:\Technion\044167 - project A\oak-d\depthai-tutorials-practice\2-ball-tracker")
ap.add_argument("-b", "--buffer", type=int, default=64,
                help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
'''greenLower = (0, 0, 157)
greenUpper = (65, 150, 242)


hava:
greenLower = (9, 90, 140)
greenUpper = (22, 167, 202)

home:
'''
greenLower = (8, 90, 151)
greenUpper = (16, 201, 255)

pts = deque(maxlen=args["buffer"])
# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	vs = VideoStream(src=1).start()
# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])
# allow the camera or video file to warm up
time.sleep(1.0)
print("slept")



x = 0
y = 0
size = 0

ball_detected = False



def task1():

    @event(robot.when_play)
    async def music(robot):
        # This function will not be called again, since it never finishes.
        # Only task that are not currenctly running can be triggered.
        print('music!')
        
        while True:
            # No need of calling "await hand_over()" in this infinite loop, because robot methods are all called with await.
            global x
            global y
            global ball_detected
            print("robot sees ball at: " +str(x) +" , " +str(y) + " ball detection " + str (ball_detected))
            
            if ball_detected:
                if x < 200:
                    await robot.turn_left(10)
                    await robot.wait(0.3)
                    
                elif x > 250:
                    await robot.turn_left(-10)
                    await robot.wait(0.3)

                if y <  200:
                    await robot.move(5) 
                
            else:
                await robot.wait(0.3)

            key = cv2.waitKey(1)
            # if the 'q' key is pressed, stop the loop
            if key == ord("q"):
                break

        await robot.stop()
    robot.play()



def Ball_Tracking():

    while True:
        global x
        global y
        global radius
        global ball_detected
	  
        in_rgb = q_rgb.tryGet()
        if in_rgb is not None:
            frame = in_rgb.getCvFrame()

        # then we have reached the end of the video
        if frame is None:
            break
        
        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None



        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            ball_detected = True

            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                #print(int(x))
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
        else:
            ball_detected = False 
        # update the points queue
        pts.appendleft(center)
        # loop over the set of tracked points
        for i in range(1, len(pts)):
            # if either of the tracked points are None, ignore
            # them
            if pts[i - 1] is None or pts[i] is None:
                continue
            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            # un-commnet next line to see trajectory 
            #cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
        # show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1)
        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break


    # if we are not using a video file, stop the camera video stream
    if not args.get("video", False):
        vs.stop()
    # otherwise, release the camera
    else:
        vs.release()
    # close all windows
    cv2.destroyAllWindows()


if __name__ == "__main__":

	# print ID of current process
    print("ID of process running main program: {}".format(os.getpid()))

	# print name of main thread
    print("Main thread name: {}".format(threading.current_thread().name))
    

	# creating threads
    t1 = threading.Thread(target=task1, name='t1')
    t2 = threading.Thread(target=Ball_Tracking, name='t2')

	# starting threads
    t2.start()
    time.sleep(1)
    t1.start()

	# wait until all threads finish
    t1.join()
    t2.join()
 