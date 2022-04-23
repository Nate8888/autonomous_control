# Shell script to open 6 terminals, go to /dev_ws/ directory and type the following commands:
# . install/setup.bash
# . /usr/share/gazebo/setup.sh

# parse argument directory
if [ $# -eq 0 ]; then
    echo "No argument supplied"
    echo "Usage: ./coolnate.sh <directory>"
    exit 1
fi

# Go to the directory of ROS2 workspace suplied on the argument
cd ~/$1/



# First terminal:
# ros2 launch ezrassor_sim_gazebo gazebo_launch.py

# Second terminal:
# ros2 launch ezrassor_sim_description spawn_ezrassor.py

# Third terminal:
# rviz2


for i in {1..8}
do
    if [ $i -eq 1 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; ros2 launch ezrassor_sim_gazebo gazebo_launch.py world:=obstacle.world; exec bash"
        sleep 10
    elif [ $i -eq 2 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; ros2 launch ezrassor_sim_description spawn_ezrassor.py y:=0; exec bash"
        sleep 10
    elif [ $i -eq 3 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; rviz2; exec bash"
    elif [ $i -eq 4 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; ros2 run autonomous_control interactor; exec bash"
        sleep 20
    elif [ $i -eq 5 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; ros2 topic pub /interact std_msgs/msg/Int32 {data:\ 2\} --times 1; exec bash"
        sleep 20
    elif [ $i -eq 6 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; ros2 run autonomous_control autonomous_control; exec bash"
    elif [ $i -eq 7 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; rqt; exec bash"
    else
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; exec bash"
    fi
    
done