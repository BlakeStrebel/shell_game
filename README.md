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

### get_cup node ###

### planning node ###

Motion planning here uses two main players - 
1) Baxter's inbuilt IK solver

2) MOVEIT! motion planning.

A node is written to call ik_service and joints required by Baxter arm are accepted into planning server and MOVE-IT! plans trajectory in joint-space to reach the desired configuration.

No scene is imported into the MOVEIT! configuration. Hence the only collision detection / avoidance it does is wrt its own parts (body, other arm).

While solving IK, the random seed 'heuristic' is used to try for a specified time if solution is not found in the first go.
