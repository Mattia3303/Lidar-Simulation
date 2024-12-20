cmake_minimum_required(VERSION 3.10)
project(NonUniformPointCloudPlugin)

# Find the required Ignition libraries
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-sensors6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-gazebo6 REQUIRED)

# Add the plugin source
add_library(NonUniformPointCloudPlugin SHARED NonUniformPointCloudPlugin.cc)

# Include directories
target_include_directories(NonUniformPointCloudPlugin PRIVATE
    ${IGNITION-PLUGIN1_INCLUDE_DIRS}
    ${IGNITION-SENSORS6_INCLUDE_DIRS}
    ${IGNITION-TRANSPORT11_INCLUDE_DIRS}
    ${IGNITION-GAZEBO6_INCLUDE_DIRS})

# Link to the required libraries
target_link_libraries(NonUniformPointCloudPlugin
    ignition-plugin1::ignition-plugin1
    ignition-sensors6::ignition-sensors6
    ignition-transport11::ignition-transport11
    ignition-gazebo6::ignition-gazebo6)
