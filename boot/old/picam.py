#!/usr/bin/env python

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size = (640,480))
 
# allow the camera to warmup
time.sleep(0.1)

rospy.init_node('picam', anonymous=True)
im_pub = rospy.Publisher("image/compressed", CompressedImage, queue_size = 1)
msg = CompressedImage()

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    if rospy.is_shutdown():
        break

    image = frame.array
 
    msg.header.stamp = rospy.Time.now()
    msg.format='jpeg'
    msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
    im_pub.publish(msg)
    #rospy.sleep(0.1)
    rawCapture.truncate(0)

