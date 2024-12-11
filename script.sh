#per vedere i messaggi della sim del lidar su ros2
ros2 topic echo /lidar/points

#Runnare il bridge tra ROS e Ignition 
ros2 run ros_gz_bridge parameter_bridge /lidar/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked

