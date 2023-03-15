# rob6_mobilebase

This repo is needed for the Lidar: https://github.com/SICKAG/sick_safetyscanners2
remember to change the ip address for the lidar and pc in the launch file
![image](https://user-images.githubusercontent.com/72868875/221827761-76bf8fb8-b73b-453e-bda0-a2229671764b.png)

install dependencies
```
sudo apt install ros-galactic-navigation2 ros-galactic-nav2-bringup ros-galactic-ros2-control ros-galactic-ros2-controllers ros-galactic-gazebo-ros2-control ros-galactic-gazebo-ros-pkgs ros-galactic-xacro ros-galactic-joint-state-publisher-gui ros-galactic-twist-mux sl

pip install opencv-python
```

optional 
```
sudo apt install ros-galactic-turtlebot3*
```

You will need this hardware interface for dynamixel motors https://github.com/dynamixel-community/dynamixel_hardware/tree/galactic
