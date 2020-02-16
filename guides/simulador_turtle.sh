# Script que limpa algumas variáveis que apontam para o robô fisico
export TURTLEBOT3_MODEL=waffle_pi
export ROS_MASTER_URI="http://localhost:11311"
unset ROS_IP
roslaunch turtlebot3_gazebo turtlebot3_world.launch
