if [ $# -eq 0 ]; then
    echo "No argument supplied"
    echo "Usage: ./coolnate.sh <directory>"
    exit 1
fi

cd ~/$1/


for i in {1..5}
do
    if [ $i -eq 1 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; ros2 launch ezrassor_sim_gazebo gazebo_launch.py; exec bash"
        sleep 10
    elif [ $i -eq 2 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; ros2 launch ezrassor_sim_description spawn_ezrassor.py y:=0; exec bash"
        sleep 10
    elif [ $i -eq 3 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; ros2 run autonomous_control autonomous_control; exec bash"
    elif [ $i -eq 4 ]; then
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; rqt; exec bash"
    else
        gnome-terminal --tab --title="ROS2 workspace" -- bash -c "cd ~/$1/; . install/setup.bash; . /usr/share/gazebo/setup.sh; echo 'Starting ROS2 workspace'; exec bash"
    fi
    
done