# Sawyer-Project
This repository contains the code to connect the CSL gamepad to a sawyer robot utilizing a custom node, as well as some recordings of usage with and without refinements to the movement for comparison.

## Running Instructions

Connect the joystick/gamepad to your computer.

### Launch the nodes:

roslaunch cu_sawyer_gamepad controller_to_arm.launch

This will start:

joy_node (publishes joystick messages to /joy)

ControllerToArm (subscribes to /joy and publishes /robot/limb/right/command_twist_stamped and /robot/gripper/command)

### Initialize the gamepad:

Press the L2 and R2 triggers simultaneously to activate the controller (as indicated by the console warning).
