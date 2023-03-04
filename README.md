This repo has differnt scripts that can be used to move the Sloth-bot around in different ways. TO run a script cd to the linux64 file of the desired script and run the executable.

climbing:
The robot can be controlled using wasd (right arm) and ijkl (left arm) to move the arms in and out and up and down. 

auto_climb:
Runs a script that steps through defined positions for a climbing gait where there are two hands on each rung. Press ENTER to step through all of the different positions and make the sloth climb. The auto_climb script uses a verbose version of the code while the funciton version has been condensed using a function to move the robot. The Makefile must be changed to run auto_climb instead of auto_climb_function. 

one_arm_climb:
Runs a script that autonomously has the robot climb up two rungs. This script uses a gait where there is only ever one arm per rung. Upon starting the script press any buttton then ENTER to create a homing position then ENTER again to have the robot climb. 

climb_down:


commanded_climb:
A combination of teleop and autonomous climb. Takes keyboard presses to trigger autonomous climb motions for the right arm and left arm going up. Press w to move the left arm up a rung and i to move the right arm up a rung. 

cv: 
