## waypoint_maker 

## waypoint_clicker 

use RViz clicked points to extract waypoints from vector map and save, needs vector map 

Publications 
- /waypoint_guide[visualization_msgs::Marker>] 

Subscriptions: 
- /clicked_point[geometry_msgs::PointStamped] 
- /vector_map_info/point[vector_map::PointArray] 
- /vector_map_info/lane[vector_map::LaneArray] 
- /vector_map_info/node[vector_map::NodeArray] 

## waypoint_creator 

Use RViz clicked points to extract waypoints from objects on RViz.  

Publications:  
- /waypoint_creator/debug/waypoints [visualization_msgs/MarkerArray] 
- /waypoint_creator/lane_array [autoware_msgs/LaneArray] 

Subscriptions:  

- /clicked_point [geometry_msgs/PointStamped] 
- /move_base_simple/goal [geometry_msgs/PoseStamped] 

## waypoint_extractor 

Subscribe to lane_array topic (from waypoint_creator) and save it to a waypoint csv file (default to home folder) 

Subscriptions:  

- /waypoint_creator/lane_array [autoware_msgs/LaneArray] 

## waypoint_loader 

see planning path 

Publications: 
- /based/lane_waypoints_raw [autoware_msgs/LaneArray] 

## waypoint_marker_publisher 

Convert lane into marker 

Subscriptions 
- light_color[autoware_msgs::TrafficLight] 
- light_color_managed[autoware_msgs::TrafficLight] 
- lane_waypoints_array[autoware_msgs::LaneArray] 
- traffic_waypoints_array[autoware_msgs::LaneArray] 
- final_waypoints[autoware_msgs::Lane] 
- closest_waypoint[std_msgs::Int32] 
- config/lane_stop[autoware_config_msgs::ConfigLaneStop] 

Publications 
- local_waypoints_mark[visualization_msgs::MarkerArray]: marker for final_waypoints 
- global_waypoints_mark[visualization_msgs::MarkerArray]: marker for lane_waypoints_array and traffic_waypoints_array 

## waypoint_replanner 
Adjust waypoints offline (resample and replan velocity) 

Subscriptions 
- /based/lane_waypoints_raw[autoware_msgs::LaneArray] 
- /config/waypoint_replanner[autoware_config_msgs::ConfigWaypointReplanner] 

Publications 
- /lane_waypoints_array [autoware_msgs/LaneArray] 

## waypoint_saver 

Subscribe to vehicle pose and speed, save into path file, and publish markers to use in RViz 

Publications 
- waypoint_saver_marker[visualization_msgs::MarkerArray] 

Subscriptions 
- current_pose[geometry_msgs::PoseStamped] 
- current_velocity[geometry_msgs::TwistStamped] 

## waypoint_velocity_visualizer 

Use pose, twist, cmd twist and waypoints to create MarkerArray for visualization in RViz 

Subscriptions: 
- lane_waypoints_array[autoware_msgs::LaneArray] 
- final_waypoints[autoware_msgs::Lane] 
- current_pose[geometry_msgs::TwistStamped] 
- current_velocity[geometry_msgs::TwistStamped] 
- twist_cmd[geometry_msgs::TwistStamped] 

Publications 
- waypoints_velocity[visualization_msgs::MarkerArray] 
