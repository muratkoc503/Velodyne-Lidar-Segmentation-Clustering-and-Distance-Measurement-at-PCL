# Velodyne-Lidar-Segmentation-Clustering-and-Distance-Measurement-at-PCL
First 3D Velodyne lidar is used with Point Cloud Library (PCL). Then, Segmentation and Clustering is processed. Finally, distance is calculated at 3d coordinate frame for each object. After, i will plan to boxing of each object.

Velodyne Lidar installation on Ros Noetic:
sudo apt update
sudo apt upgrade
sudo apt-get install ros-noetic-velodyne-simulator

Then run to example:
rosrun my_pcl_tutorial example

Output: Distance Measurement of Each Object

![pcl](https://user-images.githubusercontent.com/75611653/146636298-3032f737-e378-4a31-9a2b-379868dac583.png)

Rviz Output:

![pcl_rviz](https://user-images.githubusercontent.com/75611653/146636310-220886fc-a925-4e9b-8d4d-38083db21298.png)
