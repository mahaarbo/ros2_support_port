# Support Port
This is a tool to migrate a ROS-Industrial robot support package to ROS 2. The porting functionality is mainly taken from [awslabs/ros2-migration-tools](https://github.com/awslabs/ros2-migration-tools). Since ROS-Industrial robot support packages are [a bit more structured](http://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages) than normal ROS packages, this package aims to give quicker porting of the support packages than [awslabs/ros2-migration-tools](https://github.com/awslabs/ros2-migration-tools). 

The core of this package is `shutil.copytree`, with custom copy functions for different filetypes. This means that when someone makes a better porting function per file, it should be relatively easy to extend this package.

# Current porting tools used
Currently the package uses the [cmake_lists_porter.py](https://github.com/awslabs/ros2-migration-tools/blob/master/porting_tools/cmake_lists_porter.py) and [package_xml_porter.py](https://github.com/awslabs/ros2-migration-tools/blob/master/porting_tools/package_xml_porter.py) with accompanying [constants.py](https://github.com/awslabs/ros2-migration-tools/blob/master/porting_tools/constants.py) and [utils.py](https://github.com/awslabs/ros2-migration-tools/blob/master/porting_tools/utils.py) from [awslabs/ros2-migration-tools](https://github.com/awslabs/ros2-migration-tools) for `CMakeLists.txt` files and `package.xml` files. It uses a modified version of [aws-robotics](https://github.com/aws-robotics/ros2-launch-file-migrator/) for porting launch files. 