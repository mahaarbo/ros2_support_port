# ROS Industrial Support Port
This is a tool to migrate a ROS-Industrial robot support package to ROS 2. The porting functionality is mainly taken from [awslabs/ros2-migration-tools](https://github.com/awslabs/ros2-migration-tools). Since ROS-Industrial robot support packages are [a bit more structured](http://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages) than normal ROS packages, this package aims to give quicker porting of the support packages than [awslabs/ros2-migration-tools](https://github.com/awslabs/ros2-migration-tools).  

# Usage
The python package creates a shell script you can run with `migrate_rosi_support`. The package also includes a class that uses porting tools similar to that of [awslabs/ros2-migration-tools](https://github.com/awslabs/ros2-migration-tools), but includes a launch file migration system. Extra conversion rules can be included for the different porting tools.

Example usage of shell script:

```
source /opt/ros/foxy/setup.bash  # To let the porting tool check installed packages
mkdir -p ~/ros2_ws/src/motoman_gp25_support  # Make the destination folder
migrate_rosi_support ~/ros1_ws/src/motoman_gp25_support ~/ros2_ws/src/motoman_gp25_support  # migrate the ros industrial support package
```

# Layout
Generally the layout and the launch files stay the same. Since we don't have global parameters anymore, the `load_#ROBOT#.launch` script now sets a `LaunchConfiguration('robot_description')` and `LaunchConfiguration('robot_description_path)` variable that can be accessed by other launch scripts. Similarly the `robot_state_publisher` has been given a special rule to take the robot description variable as a parameter on launch. 

Porting tools only handle `CMakeLists.txt`, `package.xml`, and `.launch` files, and everything else is just transferred directly.

# Report
The resulting package has a `porting_report.txt` where information is given about problematic xml elements in the launch files, which files were transferred without modifications, what may need to be transferred by hand, packages that may be missing, renamings that have happened, CMake commands that have been removed, etc. 

