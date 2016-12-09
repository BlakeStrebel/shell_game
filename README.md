# Shell Game Project #
Northwestern University ME495: Embedded Systems in Robotics (Fall 2016)
Team members: Elton Cheng, Adam Pollack, Blake Strebel, Vismaya Walawalkar
Instructor: Jarvis Schultz

## Overview ##
The purpose of this project was for Baxter to play a [shell game](https://en.wikipedia.org/wiki/Shell_game) with the user. The game involves tracking the location of a hidden 'treasure' under three identical cups. The user shows the treasure to Baxter, places it under one of the cups, and shuffles them randomly on the workspace. Baxter uses a camera in one of his hands to track the cups, and, whenever the user is done shuffling, he picks up the cup containing the treasure.

[Link to demo video](https://youtu.be/6UPHq3FVivk)

![overview](https://github.com/BlakeStrebel/shell_game/blob/master/images/demo_image.png)

## Computer Vision ##

### cup_tracking node ###

### find_treasure node


## Motion Planning
Two main components of motion planning that we had to solve for was 1) Turning the pixel location of the cup with the treasure seen by our CV software into real world coordinates that Baxter's right arm can move to, and 2) Using an IK service to find the joint angles needed to get to that real world coordinate, as well as finding the ideal motion path for Baxter's right arm to take.

We will describe below each of these components and how they were implemented for this project.

### Finding the Location of the Cup in the 3D World###
The [get_cup.py] file is responsible for creating the ROS node, get_cup. This node is mainly responsible for converting the ROS topic, /treasure_cup_location (Point message that describes the pixel location of the cup containing the treasure), into (x,y) points in the real world relative to Baxter.

Our first test code to try to solve this problem was to use the convertTo3D function. This function took the pixel info and adjusted the pixel values based on the calibration of the camera. After this adjustment, we used the projectPixelTo3DRay function found in the image_geometry package, to find a ray that would point to a 3D point in the world relative to the location of the camera. Applying these values to the current location of the camera will give us the position of the cup relative to Baxter.

However, this first generation of code did not work as well as intended, as there was always a slight offset to the cup, which was dependent on where the cup was located relative to the camera. I believe that this occurred due to how the cups and CV algorithm were used. Since the red cups were completely red, instead of just the top of the cup, the CV algorithm would return the center of the whole red shape seen instead of the top of the cup. This offset from the center of the cup would therefore cause an offset in the 3D location of the cup.

The final generation of this calculation can be found in the testnode() function. The offsets and pixel size were hard-coded in, as the camera would be static during the whole tracking process. As a result of this, we were able to find the 3D coordinates more accurately, but only if the environment stays the same (ie, height of the table doesnt change, different camera poses, etc.)

### Motion of the Arm ###
Motion planning here uses two main players -
1) Baxter's inbuilt IK solver

2) MOVEIT! motion planning.

A node is written to call ik_service and joints required by Baxter arm are accepted into planning server and MOVE-IT! plans trajectory in joint-space to reach the desired configuration.

No scene is imported into the MOVEIT! configuration. Hence the only collision detection / avoidance it does is wrt its own parts (body, other arm).

While solving IK, the random seed 'heuristic' is used to try for a specified time if solution is not found in the first go.


[get_cup.py]:<https://github.com/BlakeStrebel/shell_game/blob/master/src/get_cup.py>
