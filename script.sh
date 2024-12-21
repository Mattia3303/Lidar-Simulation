
#1)
cd ./Non_Uniform_PointCloud_Plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build

#2)
ros2 run ros_gz_bridge parameter_bridge /lidar/non_uniform_scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked
        #old:
        ros2 run ros_gz_bridge parameter_bridge /lidar/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked


#per vedere i messaggi della sim del lidar su ros2
ros2 topic echo /lidar/points

#Fixed frame per rviz
[cmp]

