# rob6_mobilebase
This is the repository for AAU Robotics bachelor project. This project aims to compete in DTUs annual competition RoboCup, using a custom mobile robotic platform, with a crustcrawler manipulator. This repository has all the code for the industrial pc running ubuntu server and ROS2 galactic. The PLC code can be found here: https://github.com/Rlilholt99/ROB6_brdkAGV_PLC  

## Install dependencies
```
sudo apt install ros-galactic-slam-toolbox ros-galactic-ros2-control ros-galactic-ros2-controllers ros-galactic-xacro ros-galactic-joint-state-publisher-gui ros-galactic-twist-mux ros-galactic-sick-safetyscanners-base ros-galactic-sick-safetyscanners2-interfaces ros-galactic-dynamixel-workbench-toolbox ros-galactic-moveit ros-galactic-bondcpp ros-galactic-behaviortree-cpp-v3 ros-galactic-test-msgs

pip install opencv-python
```


In a ros2 workspace build navigation2, dynamixel_interface and sicksafety_scanners2 from source 
```
mkdir -p ~/ros2_ws/src
cd ros2_ws/src
git clone https://github.com/ros-planning/navigation2 -b galactic
git clone https://github.com/rst-tu-dortmund/teb_local_planner.git -b galactic
git clone https://github.com/rst-tu-dortmund/costmap_converter.git -b ros2 
git clone https://github.com/SICKAG/sick_safetyscanners2.git 
cd ..
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro galactic
colcon build --symlink-install
# remember to source the workspace !
```

## Install this repository
Since you are likely going to edit the source code for this at some point we recommend that it is built in a separate workspace.
```
mkdir -p ~/galactic/src
cd ~/galactic/src
git clone https://github.com/Lass6230/rob6_mobilebase
git clone https://github.com/dynamixel-community/dynamixel_hardware -b galactic
cd ..
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro galactic
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# remember to source the workspace!
```



For simulation (optional)
```
sudo apt install ros-galactic-turtlebot3* ros-galactic-gazebo-ros2-control ros-galactic-gazebo-ros-pkgs
```

Remember to change the ip address of the lidar in the launch file "sick_safetyscanners2_launch.py"
![image](https://user-images.githubusercontent.com/72868875/221827761-76bf8fb8-b73b-453e-bda0-a2229671764b.png)