# Spatial-cognitive-exploration-algorithm
This repository contains a novel 2D exploration algorithm that automates indoor-mapping of unknown enviornment. The algorithm uses a 2D-LiDAR to locate points (viewpoints) suitable to perform scanning. The viewpoints are detetced using a Gaussian process regression model. The algorithm is designed for real-estate industry to develop VR tours and 3D reconstruction of Dutch houses.

### Project content
The project contains the following packages:

* **exploration_node**: Package containing the exploration algorithm.
* **jackal**: Contains the Jackal robot simulation model and Gazebo environment models (https://github.com/jackal).
* **mrpt_slam**: Contains the implementation of ICP SLAM ROS package (https://github.com/mrpt-ros-pkg/mrpt_slam).
* **point_cloud_convertor**: ROS package used to convert pointcloud1 to pointcloud2 (https://github.com/pal-robotics-forks/point_cloud_converter).
## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### System requirement
```
Ubuntu 16.04
python 2.7
ROS kinteic
```
### Download from source
Run the following commands to clone the project on your workspace
```
my_catkin_workspace/src$ git clone git@github.com:atulhari/Spatial-cognitive-exploration-algorithm.git
my_catkin_workspace/src$ cd ..
my_catkin_workspace$ catkin_make
```
Additional packages
The python pakcages required for running the algorithm are as follows:
```
matplotlib==2.2.4
simplification==0.4.2
Shapely==1.6.4.post2
tqdm==4.35.0
numpy==1.15.3
scikit_image==0.14.2
scipy==1.2.2
tensorflow_gpu==1.12.0
gpflow==1.5.1
polygon_Mapper==0.1.1
skimage==0.0
tensorflow==2.1.0
```
The requirements can be installed by
```
pip install -r requirements.txt.
```
### Running
```
$ roslaunch exploration_node bringup.launch
```
For SCEAM algorithm
```
$ roslaunch exploration_node SCEAM.launch
```
For running iSCEAM algorithm
```
$ roslaunch exploration_node iSCEAM.launch
```
## Authors
**Atul hari**

Robotics and mechatronics lab, University of Twente

## Acknowledgments
TSP solver using ant-colony-optimization: Developed by Rochak Gupta https://github.com/rochakgupta/aco-tsp was used in the planner.
