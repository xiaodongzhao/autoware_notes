## amathutils_lib 

## autoware_build_flags 

## autoware_health_checker 

## emergency_handler 

## gnss 

## lanelet2_extension 

##  libvectormap 

## libwaypoint_follower 

## map_file 

More details at map_file package. 

### points_map_loader 

Read and publish a map (pcd) file or a list of pcd files. 

Publications: 

- /pmap_stat [std_msgs/Bool] 
- /points_map [sensor_msgs/PointCloud2] 

### vector_map_loader 

Loads vector_map file/files and publish the map data as vector_map_msgs messages. 

Publications: 
- vector_map [visualization_msgs::MarkerArray]: map markers 
- vmap_stat [std_msgs::Bool]: true indicates map marker has been published. 
- vector_map_info/* [vector_map_msgs]: depedending input file name, this node will publish different topics, for details, refer to source code. 

Subscriptions: 
- vector_map_info/* messages: the same topics are subscribed by an internal VecterMap object to create vector map. 

### lanelet2_map_loader 

lanelet2_map_loader loads Lanelet2 file and publish the map data as autoware_lanelet2_msgs/MapBin message. The node projects lan/lon coordinates into MGRS coordinates. 

Publications: 
- /lanelet_map_bin [autoware_lanelet2_msgs/MapBin] : Binary data of loaded Lanelet2 Map. 
### lanelet2_map_visualization 

lanelet2_map_visualization visualizes autoware_lanelet2_msgs/MapBin messages into visualization_msgs/MarkerArray. 

Subscriptions: 
- /lanelet_map_bin [autoware_lanelet2_msgs/MapBin] : binary data of Lanelet2 Map 

Publications: 
- /lanelet2_map_viz [visualization_msgs/MarkerArray] : visualization messages for RVIZ 

### points_map_filter 

points_map_filter_node subscribe pointcloud maps and current pose, the node extract pointcloud near to the current pose. 

Subscriptions: 
- /points_map [sensor_msgs/PointCloud2] : Raw pointcloud map. This topic usually comes from points_map_loader. 
- /current_pose [geometry_msgs/PoseStamped] : Current pose of the car. This topic usually comes from pose_relay node. 

Publications: 
- /points_map/filtered [sensor_msgs/PointCloud2] : Filtered pointcloud submap. 

## object_map 

## op_planner 

## op_ros_helpers 

## op_simu 

## op_utility 

## ros_observer 

## vector_map 

## vector_map_server 