# Shell Game Project #
Northwestern University ME495: Embedded Systems in Robotics (Fall 2016)
Team members: Elton Cheng, Adam Pollack, Blake Strebel, Vismaya Walawalkar
Instructor: Jarvis Schultz

## Overview ##
The purpose of this project was for Baxter to play a [shell game](https://en.wikipedia.org/wiki/Shell_game) with the user. The game involves tracking the location of a hidden 'treasure' under three identical cups. The user shows the treasure to Baxter, places it under one of the cups, and shuffles them randomly on the workspace. Baxter uses a camera in one of his hands to track the cups, and, whenever the user is done shuffling, Baxter picks up the cup containing the treasure.

[Link to demo video](https://youtu.be/6UPHq3FVivk)

![overview](https://github.com/BlakeStrebel/shell_game/blob/master/images/demo_image.png)

## Computer Vision ##

### find_treasure node ###
Description: [find_treasure.py](https://github.com/BlakeStrebel/shell_game/blob/master/src/find_treasure.py) is responsible for finding the location of the treasure at any point during the game. This allows Baxter to identify if the user has switched the treasure from one cup into another at any point during the came. Treasure is defined as any green object placed in Baxter's field of view. This description could be adjusted to allow Baxter to identify different colors, shapes, etc.
Subscribes to:
* `/cameras/left_hand_camera/image`
Publishes:
* `/treasure_location`: [treasure_location](https://github.com/BlakeStrebel/shell_game/blob/master/msg/Treasure.msg) is a custom message that contains a flag indicating if the treasure is currently in Baxter's field of view. It also contains two floats representing the current location of the treasure. If the treasure is not currently visible, it publishes its last known position.

### cup_tracking node ###
Description: [cup_tracking.py](https://github.com/BlakeStrebel/shell_game/blob/master/src/cup_tracking.py) finds the location of the three cups and keeps track of which one is which. This allows Baxter to track the cups and associate the treasure with one of them  
Subscribes to:
* `/cameras/left_hand_camera/image`
* `/treasure_location`  
Publishes to:
* `/treasure_cup_location`
* `/robot/xdisplay`

This node finds the location of the 3 red cups in the field of view. Once the 3 cups are found, the node tracks each cup by noting the position of each cup during each iteration and finding the cup in the next iteration which is closest. To determine which cup contains the treasure, this node subscribes to `/treasure_location` (published by the find_treasure node). `/treasure_location` is a custom message consisting of a boolean value which defines whether or not the treasure is visible to Baxter, a x value, and a y value. The cup_tracking node checks the boolean value, and, if the visibility of the treasure goes from true to false, it finds the nearest cup to that position and marks it as containing the treasure. Finally, the cup_tracking node publishes the location of the cup that contains the treasure in pixel coordinates.

## Motion Planning ##
Motion planning nodes were responsible for:
* Converting the pixel location of the cup with the treasure Baxter's coordinate frame
* Determining the ideal path and joint angles needed to move Baxter's arm to pick up the cup

### get_cup node ###
Description: [get_cup.py](https://github.com/BlakeStrebel/shell_game/blob/master/src/get_cup.py) is responsible for converting the pixel location of the cup containing the treasure into real world coordinates usable by Baxter.
Subscribes to:
* `/cameras/left_hand_camera/camera_info`
* `/robot/limb/left/endpoint_state`
* `/robot/diigital_io/right_button/state`
* `/treasure_cup_location`

This code utilizes the pixel conversion equation found in the [visual servoing example](http://sdk.rethinkrobotics.com/wiki/Worked_Example_Visual_Servoing) for Baxter:

B = (Pp – Cp) * cc * d + Bp + Go

where:
    B = Baxter coordinates
    Pp = pixel coordinates
    Cp = centre pixel coordinates
    Bp = Baxter pose
    Go = gripper offset
    cc = camera calibration factor
    d = distance from table

### planning node ###
Description: [planningnode.py](https://github.com/BlakeStrebel/shell_game/blob/master/src/planningnode.py) uses Baxter's built in inverse kinematics solver and [MOVEIT!](http://moveit.ros.org/) motion planning to make Baxter's arm pick up the cup. This node calls Baxter's `ik_service` in order to determine the joint angles needed to achieve a desired end-effector position. Joint angles required are then fed into the planning server and `MOVE-IT!` plans a trajectory in joint-space to reach the desired configuration.

Note: No scene is imported into the MOVEIT! configuration. Therefore, the only collision detection simulated is to prevent Baxter from colliding with himself.While solving inverse kinematics, the random seed heuristic is used if a solution is not found the first time.
