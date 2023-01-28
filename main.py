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
import art
import keyboard

#connect to the robot via Create3 SDK kit
robot = Create3(Bluetooth())


#create a pipeline to connect to the camera.
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


robot_x = 0
robot_y = 0


STANDBY_X = 50
STANDBY_Y = 0

# x and y will be the coordinates of the ball located in the camera view 
ball_x = 0
ball_y = 0

# radius is the radius of the ball detected
radius = 0
MIN_RADUIS = 10

# Ball_detected if a flag if there is a ball detected or not.
ball_detected = False

ball_in_place = False


# state will save at witch state the robot is.
# the values that state can get are the following strings:
# 0-> InitialSetup  -  here the robot must be set up it is expected that the robot before activating must be on the docking station at the right position
# 1-> StandBy  -  here the robot should stand in stand by mode and must not intefeir in the game being played.
# 2-> ApproachingBall  -  a ball has been detected and now the robot must position himeself in a way that allows him to pick the ball up.
# 3-> CollectingBall  -  the ball is located in the desired location relative to the robot so now the "collecting" process must start
# 4-> Done  -  here the robot has finished and is expected to go to the docking station and dock.    
stete = None


def task1():

    @event(robot.when_play)
    async def music(robot):
        # This function will not be called again, since it never finishes.
        # Only task that are not currenctly running can be triggered.
        
        while True:
            # No need of calling "await hand_over()" in this infinite loop, because robot methods are all called with await.
            global ball_x
            global ball_y
            global ball_detected
            global state
            global robot_x, robot_y

            #print("robot sees ball at: " +str(ball_x) +" , " +str(ball_y) + " ball detection " + str (ball_detected))
            pos = await robot.get_position()

            robot_x = pos.x
            robot_y = pos.y
            if state == 'idle':
                await robot.wait(0.2)

            if state == 'InitialSetup':
                print(art.text2art('LET\'S GO!'))
                await robot.undock()
                await robot.turn_left(180)
                await robot.wait(2) 

            elif stete == 'StandBy':
                await robot.navigate_to(STANDBY_X, STANDBY_Y)
                await robot.wait(0.3) 

            elif stete == 'ApproachingBall':
                if ball_x < 200:
                    await robot.turn_left(10)
                    await robot.wait(0.3)

                elif ball_x > 250:
                    await robot.turn_right(10)
                    await robot.wait(0.3)

                if ball_y < 200:
                    await robot.move(5)

                
            elif stete == 'CollectingBall':
                pass
            
            # stete == 'Done'
            else:
                print(art.text2art('Done!  Now Docking '))
                await robot.dock()
                await robot.stop()

    robot.play()



def Ball_Tracking():

    while True:
        global ball_x
        global ball_y
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
            ((ball_x, ball_y), radius) = cv2.minEnclosingCircle(c)
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


def mainThread():
    global state, ball_detected, ball_in_place, ball_x, ball_y, radius, MIN_RADUIS, MIN_X, MAX_X, MIN_Y, MAX_Y
    
    state = 'idle'
    
    while True:
        if keyboard.read_key() == "i":
            state = 'InitialSetup'
            time.sleep(3)
            break
    
    while True:

        if  ball_detected:
            if ball_x > MIN_X and ball_x < MAX_X and ball_y > MIN_Y and ball_y < MAX_Y and radius > MIN_RADUIS:
                ball_in_place = True
                state = 'CollectingBall'
            else:
                state = 'ApproachingBall'
        else:
            state = 'StandBy'

        # if the 'q' key is pressed, stop the loop
        if keyboard.read_key() == "d":
            state = 'Done'
        

if __name__ == "__main__":

	# print ID of current process
    print("ID of process running main program: {}".format(os.getpid()))

	# print name of main thread
    print("Main thread name: {}".format(threading.current_thread().name))
    

	# creating threads
    t1 = threading.Thread(target=task1, name='t1')
    t2 = threading.Thread(target=Ball_Tracking, name='t2')
    t3 = threading.Thread(target=mainThread,name='mainThread')

	# starting threads
    t3.start()
    t2.start()
    time.sleep(1)
    t1.start()

	# wait until all threads finish
    t1.join()
    t2.join()
 