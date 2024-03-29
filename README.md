# UR Manipulator Robots Controller

This software has been developed for controlling manipulator robots in the [UR series](https://www.universal-robots.com/), such as UR3(e), UR5(e) and UR10(e). The software facilitates communication with the robot via [ROS (Robot Operating System)](https://www.ros.org/) and can control either the physical UR robots or the simulation one in [Gazebo](https://gazebosim.org/home).

In addition, this repository includes MATLAB scripts that can also be utilized for controlling UR robots (without the necessity of using Linux OS or undergoing a build process like C++). These scripts have been crafted for teaching and learning purposes within the context of the 41013 Industrial Robotics course at the [University of Technology Sydney](https://www.uts.edu.au/).

## Prerequisite
1. A Linux machine setup with ROS. It is recommended to use Ubuntu 20.04 with [ROS noetic](https://wiki.ros.org/noetic); however, older versions of Ubuntu should function effectively as well.

2. ROS driver package for Universal Robots robotic arms: [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)


## Installation
1. [Create catkin workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace), then clone the repository and the [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) into the `src` directory of your catkin_ws:
```bash
cd ~/catkin_ws/src
git clone https://github.com/mintnguyenn/UR-Cobot-Controller.git
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
```
2. Build the software
```bash
cd ..
catkin_make
```
3. **Remember** to source your workspace
```bash
source devel/setup.bash
```

## Usage

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
