# Shell Game Project #
Northwestern University ME495: Embedded Systems in Robotics (Fall 2016)
Team members: Elton Cheng, Adam Pollack, Blake Strebel, Vismaya Walawalkar  
Instructor: Jarvis Schultz

## Overview ##
The purpose of this project was for Baxter to play a [shell game](https://en.wikipedia.org/wiki/Shell_game) with the user. The game involves tracking the location of a hidden 'treasure' under three identical cups. The user shows the treasure to Baxter, places it under one of the cups, and shuffles them randomly on the workspace. Baxter uses a camera in one of his hands to track the cups, and, whenever the user is done shuffling, the user presses a button and Baxter picks up the cup containing the treasure.

[Link to demo video](https://youtu.be/6UPHq3FVivk)

![overview](https://github.com/BlakeStrebel/shell_game/blob/master/images/demo_image.png)

## Computer Vision ##

### cup_tracking node ###
Description: The cup_tracking node finds the location of the three cups and keeps track of which one is which. This allows Baxter to track the cups and associate the treasure with one of them  
Subscribes to:  
`/cameras/left_hand_camera/image`
`/treasure_location`  
Publishes to:  
`treasure_cup_location`
`/robot/xdisplay`  

This node finds the location of the 3 red cups in the field of view. Once the 3 cups are found, the node tracks each cup by noting the position of each cup during each iteration and finding the cup in the next iteration which is closest. To determine which cup contains the treasure, this node subscribes to `/treasure_location` (a topic published by the find_treasure node). `/treasure_location` publishes a custom message consisting of a boolean value which defines whether or not the treasure is visible to Baxter, an x value, and a y value. The cup_tracking node checks the boolean value and if the visibility of the treasure goes from true to false, cup_tracking finds the nearest cup to that position and marks it as containing the treasure. Finally, the cup_tracking node publishes the location of the cup that contains the treasure.

### find_treasure node


## Motion Planning

### get_cup node ###

### planning node ###

Motion planning here uses two main players -
1) Baxter's inbuilt IK solver

2) MOVEIT! motion planning.

A node is written to call ik_service and joints required by Baxter arm are accepted into planning server and MOVE-IT! plans trajectory in joint-space to reach the desired configuration.

No scene is imported into the MOVEIT! configuration. Hence the only collision detection / avoidance it does is wrt its own parts (body, other arm).

While solving IK, the random seed 'heuristic' is used to try for a specified time if solution is not found in the first go.
