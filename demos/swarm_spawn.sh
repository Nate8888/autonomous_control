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


for i in {1..5}
do
    # Add 8 tabs to the terminal and run the following commands in each of them:
    # . install/setup.bash
    # . /usr/share/gazebo/setup.sh
    # and then run the following commands:
    # if i == 1, then run 'ros2 launch ezrassor_sim_gazebo gazebo_launch.py'
    # if i == 2, then run 'ros2 launch ezrassor_sim_description spawn_ezrassor.py robot_name:=ezrassor5 y:=0'
    # if i == 3, then run 'ros2 launch ezrassor_sim_description spawn_ezrassor.py robot_name:=ezrassor4 y:=2'
    # if i == 4, then run 'ros2 launch ezrassor_sim_description spawn_ezrassor.py robot_name:=ezrassor3 y:=4'
    # if i == 5, then run 'ros2 launch ezrassor_sim_description spawn_ezrassor.py robot_name:=ezrassor2 y:=6'
    # if i == 6, then run 'ros2 launch ezrassor_sim_description spawn_ezrassor.py robot_name:=ezrassor y:=8'

    if [ $i -eq 1 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; ros2 launch ezrassor_sim_gazebo gazebo_launch.py; exec bash"
        sleep 10
    elif [ $i -eq 2 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; ros2 launch ezrassor_sim_description spawn_ezrassor.py robot_name:=ezrassor y:=0; exec bash"
        sleep 20
    elif [ $i -eq 3 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; ros2 launch ezrassor_sim_description spawn_ezrassor.py robot_name:=ezrassor2 y:=2; exec bash"
        sleep 20
    elif [ $i -eq 4 ]; then
        # ros2 run autonomous_control interactor
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; ros2 run autonomous_control interactor; exec bash"
        sleep 20
    elif [ $i -eq 5 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; ros2 topic pub /interact std_msgs/msg/Int32 {data:\ 3\} --times 1; exec bash"
    else
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; exec bash"
    fi

    # Wait 20 seconds for the terminal to run
    
done