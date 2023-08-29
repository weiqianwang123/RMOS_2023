# RMOS


## Environment
    -ros2-foxy
    -opencv4.4.0
    -Eigen
    -camera_info_manager(foxy)
## Sensors
    -Daheng camera
    -IMU(C_board from DJI)
## Things before running successfully
    -Change the address of 3rdparty/lib in almost  every CMakeLists.txt

## Structure

- camera_node： daheng_cam   || command：`ros2 run rmos_cam daheng_camera`
- detect_node：basic_detector || command： `ros2 run rmos_detector basic_detector`
- process_node：processer     ||  command：`ros2 run rmos_processer processer`
- communication_node ： can_comm      ||  command：`ros2 run rmos_transporter can_comm`

## Run the whole project
`source install/setup.bash`

`ros2 launch rmos_bringup normal_aim.launch.py`


## Configures needed to be modified before running

- The time of exposure and the gain should be modified in rmos_bringup/configure/daheng_camera.xml

- The parameter of camera used for the PNP should be modified in rmos_bringup/configure/daheng_cam_info.yaml

- The offset from camera to IMU should be modified in basic_detector.cpp

- The time offset for the synchronization of sensor data should be modified in can_comm_node.cpp

-  Others are in the specific configure file under Algorithm/configure/.

  

## How to debug efficiently

- In the "Algorithm/configure/Debug" directory, there is a parameter file named "debug.xml". This file displays the supported debug options and allows for real-time modification of these options during program execution. If the first option "contest" is enabled, the program will enter competition mode, and all display of debug information will be turned off.
- Other debug information can be viewed using the "ros2 topic echo" command.
- Image information can be viewed using "rqt," including the original image, the image with highlighted armor panels, and the binary image.




## Tips

### if running unsuccessfully

-   Unplug and plug the camera cable.
-   Check if there is IMU data, unplug and plug it if needed.
-   Reboot the entire vehicle.
-   Run each node separately.
-    If you encounter a "file too short" error, please delete the "build," "log," and "install" folders, and then recompile.
-   Check for any zombie topics. For reference, please see the article: How do you kill the zombie node in ROS2 on local?







## TODO

- The dataset lacks category number 5.
- Merge with the long-range shooting code.





#### 

#### 
