# Human Following ROS Robot

### This package implements a human detection and tracking ROS robot.
This robot will follow a human, movement controls are derived from position of detected human in the image from video stream.
HOG algorithm from opencv package is used for the human detection part.
Kinect V1 is used as an RGB Camera.

Corresponding Arduino Code for the robot can be found at : https://github.com/mijazm/ROS_Arduino_PID

You will need to install a library and stack for Kinect V1 called "libfreenect".
Read this blog to know how to do that : https://aibegins.net/2020/11/22/give-your-next-robot-3d-vision-kinect-v1-with-ros-noetic/