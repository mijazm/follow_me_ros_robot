#!/usr/bin/env python3

# This code uses camera stream to detect humans and generate twist messages to control a robot.
# The robot will try to follow the detected human.
#Author: Mijaz Mukundan
#References:
# 1. http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# 2. https://data-flair.training/blogs/python-project-real-time-human-detection-counting/
# 3. https://www.pyimagesearch.com/2015/11/16/hog-detectmultiscale-parameters-explained/

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

# Adjust these parameters experimentally
MINIMUM_CONFIDENCE = 1.0 #Minimum confidence value in detection of human for the robot to activate 
DESIRED_DETECTION_AREA = 29000 # This will be used a reference, when area of the bounding box is this value
                               # the robot will ideally stop moving, the robot will move to keep the detected area
                               # near to this value

HOGCV = cv2.HOGDescriptor()
HOGCV.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
bridge = CvBridge()

rospy.init_node('pid_publisher')

image_pub = rospy.Publisher("/eyes_of_robot",Image,queue_size=1)

    # start publisher of cmd_vel to control robot
ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

def callback(data):
  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)
  
  cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
  im_h, im_w = cv_image_gray.shape
  
  # detect people in the image
  # returns the bounding boxes for the detected objects
  boxes, weights = HOGCV.detectMultiScale(cv_image_gray, winStride = (8, 8))

  if len(boxes)>0:
    #The robot will choose to follow the detected human with greatest confidence value
    max_confidence_index = np.argmax(weights)
    
    x,y,w,h = boxes[max_confidence_index]
    weight = weights[max_confidence_index]

    if weight >= 1.0:
      print(w*h)
      # display the detected boxes in the colour picture
      cv2.rectangle(cv_image, (x, y), (x+w, y+h),(0, 255, 0), 2)
      
      #Calculate the centroid error, i.e. how off-centre is the detected human
      centroid = np.array([x+w/2,y+h/2])
      image_centre = np.array([im_w/2,im_h/2])
      error_in_x = image_centre[0] - centroid[0]
      
      #Calculate area error, it will give an indication of how far the human is from the robot
      desired_area = 29000
      area_error = desired_area - w*h
      # font
      font = cv2.FONT_HERSHEY_SIMPLEX
      # fontScale
      fontScale = 0.5
      # Blue color in BGR
      color = (255, 0, 0)
      # Line thickness of 2 px
      thickness = 1
      # Using cv2.putText() method
      cv2.putText(cv_image, 'Area:'+str(w*h), (int(x+w/2),int(y+w/2)), font, 
                      fontScale, color, thickness, cv2.LINE_AA)
      cv2.putText(cv_image, 'Confidence:'+str(weight), (int(x+w/2),int(y+w/2+30)), font, 
                      fontScale, color, thickness, cv2.LINE_AA)
      cv2.putText(cv_image, 'Error in x:'+str(error_in_x), (int(x+w/2),int(y+w/2+60)), font, 
                      fontScale, color, thickness, cv2.LINE_AA)
      cv2.putText(cv_image, 'Area Error:'+str(area_error), (int(x+w/2),int(y+w/2+90)), font, 
                      fontScale, color, thickness, cv2.LINE_AA)

      # add linear x and angular z from detection parameters
      # Create Twist message
      twist = Twist()
      twist.linear.x =  area_error/desired_area
      twist.angular.z = error_in_x/(im_w/2)
      # publish cmd_vel
      ctrl_pub.publish(twist)
    else:
      twist = Twist()
      twist.linear.x = 0
      twist.angular.z = 0
      # publish cmd_vel
      ctrl_pub.publish(twist)
  else:
      twist = Twist()
      twist.linear.x = 0
      twist.angular.z = 0
      # publish cmd_vel
      ctrl_pub.publish(twist)

  try:
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image))
  except CvBridgeError as e:
    print(e)

#Subscribe to the image from Kinect V1
image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,callback)


r = rospy.Rate(10)

while not rospy.is_shutdown():
  r.sleep()