# gr20_mobile_robotics

This radar SLAM project was built based on graph SLAM, which is commonly used for LiDAR SLAM. This repo also includes a framework for radar odometry prior calculator that can be integrated in the to the SLAM packaged as a velocity prior, though the results, ultimately, weren't too meaningful. This setup was run and tested on Ubuntu 22.04 with ROS2 Humble.

## Build instructions for this repo
1. Clone/download this repo
2. Make a workspace for this project and probably good idea to have a separate conda environment.
3. Clone [g2o](https://github.com/RainerKuemmerle/g2o) globally, and install it into your conda environment (do not track g2o in git)
4. Navigate into the workspace and make sure it has the following structure, after which run the commands:
```
cd ~/<repo_ws>
rosdep install --from-paths src --ignore-src -r -y
```
Build this project:
```
rosdep install --from-paths src --ignore-src -r -y --skip-keys=libg2o
```

## Running the build and demo instructions
To run the demo directly with this ROS2 Lidar SLAM build, you will need the HDL400 dataset bagfile. Included is the link to the Radar bagfiles as well:
1. [Lidar Bag Download link](https://zenodo.org/record/6960371) - This is an outdoor dataset for Lidar SLAM specifically.
2. [Radar Bag Download link](https://cloud.oru.se/s/rHbRad83A764nmx) - This link has the forest and mine Radar bagfiles.
Note: These bagfiles are for ROS1, so we will need to convert them to ROS2 format first

To convert the bagfile (.bag) to be compatible with ROS2 we will need to run the following commands:
```
cd ~/<path to the .bag file>
rosbags-convert --src <bagfile_name>.bag --dst <folder_name_for_converted_file>
```
Now you should see a new folder with the converted .db3 file.

Now with the build set up in the last step, to run the demo we will need two terminals.

Terminal 1:
```
cd ~/<repo ws>
source install/setup.bash
ros2 launch lidarslam lidarslam.launch.py
```
Once the launch file is running in the first terminal, RViz should open up. For the sake of demostration, let's work with the HDL400 bagfile (assuming it is converted to .db3).

Terminal 2:
```
cd ~/<folder w/ the hdl400.db3 file>
ros2 bag play hdl400.db3
```

If everything is done correctly you can see the results in RViz:
![image](https://github.com/user-attachments/assets/e0bb4986-7b53-4403-b346-f2f2d21ab75b)

## Radar Odom Setup
1. The odom is set up and run just like any other ROS2 package:
```
cd ~/<repo_ws>
colcon build
source install/setup.bash
ros2 launch radar_odom run.py
```

Once the node is running, it will be waiting for the defined imu and radar topics (the defaults are /imu/vectornav and /radar_pcl_enchanced). To change these to the ones you are trying to work with, navigate to:

1. The config files - change the imu and radar topic names to match the ones you are working with.
2. radar_pcl_processor.cpp - change the imu and radar topic names here as well

