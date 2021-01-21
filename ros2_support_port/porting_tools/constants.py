# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.

PACKAGE_XML_ELEMENT_ORDER = [
    "name",
    "version",
    "description",
    "author",
    "maintainer",
    "license",
    "url",
    "buildtool_depend",
    "build_depend",
    "build_export_depend",
    "depend",
    "exec_depend",
    "test_depend",
    "member_of_group",
    "doc_depend",
    "conflict",
    "replace",
    "export",
]

CMAKE_LISTS_RENAMED_VARIABLES = {
    "${CATKIN_DEVEL_PREFIX}/": "",
    "${CATKIN_GLOBAL_BIN_DESTINATION}": "bin",
    "${CATKIN_GLOBAL_INCLUDE_DESTINATION}": "include",
    "${CATKIN_GLOBAL_LIB_DESTINATION}": "lib",
    "${CATKIN_GLOBAL_LIBEXEC_DESTINATION}": "lib",
    "${CATKIN_GLOBAL_SHARE_DESTINATION}": "share",
    "${CATKIN_PACKAGE_BIN_DESTINATION}": "bin",
    "${CATKIN_PACKAGE_INCLUDE_DESTINATION}": "include/${PROJECT_NAME}",
    "${CATKIN_PACKAGE_LIB_DESTINATION}": "lib",
    "${CATKIN_PACKAGE_SHARE_DESTINATION}": "share/${PROJECT_NAME}",
    "CATKIN_ENABLE_TESTING": "BUILD_TESTING"
}

# Commands that are not used by ROS2
CMAKE_LISTS_DELETED_COMMANDS = [
    "catkin_package",
    "roslaunch_add_file_check"
]

# List of ROS packages that have been renamed in ROS 2
RENAMED_ROS_PACKAGES = {
    "tf": "tf2",
    "roscpp": "rclcpp",  # This shouldn't be here anyways
    "rospy": "rclpy",  # This shouldn't be here anyways
    "nodelet": "rclcpp",  # This shouldn't be here anyways
    "message_generation": "rosidl_default_generators",  # Shouldn't be here
    "message_runtime": "rosidl_default_runtime",  # Shouldn't be here
    "rosconsole": "ros2_console",  # Until ROS 2 replacement exists
    "rviz": "rviz2",
    "catkin": "ament_cmake"  # Well.. it's not exactly a rename, but good enough
}

# List of packages that do not need to be found by CMake
NO_CMAKE_FIND_PACKAGES = [
    "rosidl_default_runtime",  # Shouldn't be here
    "roslaunch",  # They're probably trying to do roslaunch file testing
    "catkin"
]

# List of packages that do not need to be found in package.xml
NO_PKG_XML_DEPENDS = [
    "roslaunch"
]
# List of executables that have likely been renamed in ROS 2
RENAMED_ROS_EXECUTABLES = {
    "rviz": "rviz2"
}