# rl_dp_5_pcl

## PCL ROS Package for the RLDP5

This ROS package provides support for the PMD Flexx2, Time-of-Flight (TOF) sensor stuck to a desk. The package includes files for interfacing with the TOF sensor, processing point cloud data, and visualizing the results.

### Package Contents:
1. Nodes:

    - pixel2pcl.py: ROS node responsible for segmenting the point cloud from the Original point cloud. The point cloud is segmented by subscribing to /roayle_cam0/point_cloud and /royale_cam0/gray_image. Using OpenCV the 4 points near black markers are taken and the point cloud is segmented.
    - compute_normals.py: Node is responsible to calculate the normals and visualize in RViz as MarkerArray. To compute the normals, Open3D module is used. 

2. Launch Files:

    - camera_driver.launch: Launch file to start the TOF sensor node and configure its parameters.

3. Rviz Config:

    - scene.rviz: RViz configuration file pre-set for visualizing the TOF sensor data with the Royale Control panel, Image, PointCloud2 types added. The rviz file is located in rl_dp_5_moveit/rviz/scene.rviz


### Using the Package:

