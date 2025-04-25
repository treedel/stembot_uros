# stembot_uros
Base packages to run stembot on ROS ecosystem. Includes the some hardware drivers, bringup, and packages for autonomous navigation.

## Installation instructions
1. Download the stembot_uros and lidar packages
```bash
git clone https://github.com/treedel/stembot_uros
cd stembot_uros
git clone https://github.com/Slamtec/sllidar_ros2 src/sllidar_ros2
```

2. Install micro_ros_agent by following the below link
- https://micro.ros.org/docs/tutorials/core/first_application_linux/

3. Install the required dependencies using rosdep
```bash
rosdep install --from-paths src --ignore-src -r -y
```

4. Build and source the installed packages
```bash
colcon build
source install/setup.bash
```

## Running the packages
    1. To perform startup of the robot and its hardware, run the bringup launch file
    ```bash
    ros2 launch stembot_bringup bringup.launch.py
    ```
    Some parameters include:
        - use_lidar
        - use_camera
        - use_rviz
        - use_range_sensor, etc.

    2. To perform perform slam, run the slam launch file
    ```bash
    ros2 launch stembot_navigation slam.launch.py
    ```
    Some parameters include:
        - use_sim_time
        - use_nav
        - use_rviz, etc.

    3. To perform perform navigation, run the navigation launch file
    ```bash
    ros2 launch stembot_navigation navigation.launch.py
    ```
    Some parameters include:
        - use_sim_time
        - map_path
        - use_rviz, etc.