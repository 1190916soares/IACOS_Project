. install/setup.bash

export TURTLEBOT3_MODEL=burger

printenv TURTLEBOT3_MODEL

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py world:=~/turtlebot3_worlds/turtlebot3_house.world