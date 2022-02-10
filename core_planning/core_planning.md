## astar_search 
## costmap_generator 
## decision_maker 
## dp_planner 
## ff_waypoint_follower 

# lattice_planner 
## lattice_velocity_set 

Publications:  
- /closest_waypoint [std_msgs/Int32] 
- /detection_range [visualization_msgs/MarkerArray] 
- /obstacle [visualization_msgs/Marker] 
- /sound_player [std_msgs/String] 
- /temporal_waypoints [autoware_msgs/Lane] 

Subscriptions:  
- /base_waypoints [autoware_msgs/Lane] 
- /config/lattice_velocity_set [autoware_config_msgs/ConfigLatticeVelocitySet] 
- /current_pose [geometry_msgs/PoseStamped] 
- /current_velocity[geometry_msgs/TwistStamped] 
- /localizer_pose [geometry_msgs/PoseStamped] 
- /obj_pose [visualization_msgs/Marker]: not used 
- /vector_map_info/area [vector_map/Area] 
- /vector_map_info/cross_walk [vector_map/CrossWalk] 
- /vector_map_info/line [vector_map/Line] 
- /vector_map_info/point [vector_map/Point] 
- /vscan_points [sensor_msgs/PointCloud2] 

 

## mpc_follower 

## op_global_planner 

generate global path from start point to target point(s) on a map. Planning cost is distance only. supports autoware vector map, and special designed .kml maps. 

Publications:  
- /lane_waypoints_array [autoware_msgs::LaneArray] 
- /global_waypoints_rviz [visualization_msgs::MarkerArray] 
- /op_destinations_rviz [visualization_msgs::MarkerArray] 
- /vector_map_center_lines_rviz [visualization_msgs::MarkerArray] 

Subscriptions:  
- /initialpose [geometry_msgs::PoseWithCovarianceStamped] 
- /move_base_simple/goal [geometry_msgs::PoseStamped] 
- /current_pose [geometry_msgs::PoseStamped] or /odom[nav_msgs::Odometry] or /can_info[autoware_can_msgs::CANInfo] 
- /current_velocity [geometry_msgs::TwistStamped] 
- /vector_map_info/*  
- /occupancy_road_status [nav_msgs::OccupancyGrid] 

## op_local_planner 

## op_simulation_package 

## op_utilities 

## pure_pursuit 

Necessary topics are: current_pose, final_waypoints, current_velocity 

Publications: 
- /angular_gravity [std_msgs/Float32] 
- /ctrl_raw [autoware_msgs/ControlCommandStamped] 
- /deviation_of_current_position [std_msgs/Float32] 
- /expanded_waypoints_mark [visualization_msgs/Marker] 
- /line_point_mark [visualization_msgs/Marker] 
- /next_target_mark [visualization_msgs/Marker] 
- /next_waypoint_mark [visualization_msgs/Marker] 
- /node_status [autoware_system_msgs/NodeStatus] 
- /search_circle_mark [visualization_msgs/Marker] 
- /trajectory_circle_mark [visualization_msgs/Marker] 
- /twist_raw [geometry_msgs/TwistStamped] 

Subscriptions: 
- /config/waypoint_follower [autoware_config_msgs/ConfigWaypointFollower] 
- /current_pose [geometry_msgs/PoseStamped] 
- /current_velocity [geometry_msgs/TwistStamped] 
- /final_waypoints [autoware_msgs/Lane] 
- state_machine_lib 

## twist_filter 

Filter input ctrl_raw and twist_raw and publish at ctrl_cmd and twist_cmd 

Publications: 
- /ctrl_cmd [autoware_msgs/ControlCommandStamped] 
- /node_status [autoware_system_msgs/NodeStatus] 
- /twist_cmd [geometry_msgs/TwistStamped] 
- /twist_filter/limitation_debug/ctrl/lateral_accel [std_msgs/Float32] 
- /twist_filter/limitation_debug/ctrl/lateral_jerk [std_msgs/Float32] 
- /twist_filter/limitation_debug/twist/lateral_accel [std_msgs/Float32] 
- /twist_filter/limitation_debug/twist/lateral_jerk [std_msgs/Float32] 
- /twist_filter/result/ctrl/lateral_accel [std_msgs/Float32] 
- /twist_filter/result/ctrl/lateral_jerk [std_msgs/Float32] 
- /twist_filter/result/twist/lateral_accel [std_msgs/Float32] 
- /twist_filter/result/twist/lateral_jerk [std_msgs/Float32] 

Subscriptions: 

- /config/twist_filter [autoware_config_msgs/ConfigTwistFilter] 
- /ctrl_raw [autoware_msgs/ControlCommandStamped] 
- /twist_raw [geometry_msgs/TwistStamped] 

## twist_gate 

Publish vehicle_cmd every loop_rate. cmd can be from: 
/remote_cmd, or twist_cmd, mode_cmd, gear_cmd, accel_cmd, steer_cmd, brake_cmd, lamp_cmd, ctrl_cmd 

Publications: 
- /ctrl_mode [std_msgs/String] 
- /node_status [autoware_system_msgs/NodeStatus] 
- /vehicle_cmd [autoware_msgs/VehicleCmd] 

Subscriptions: 
- /accel_cmd [autoware_msgs/AccelCmd] 
- /brake_cmd [autoware_msgs/BrakeCmd] 
- /config/twist_filter [autoware_config_msgs/ConfigTwistFilter] 
- /ctrl_cmd [autoware_msgs/ControlCommandStamped] 
- /decision_maker/state [/decision_maker/state] 
- /emergency_velocity [autoware_msgs/VehicleCmd] 
- /gear_cmd [tablet_socket_msgs/gear_cmd] 
- /lamp_cmd [autoware_msgs/LampCmd] 
- /mode_cmd [tablet_socket_msgs/mode_cmd] 
- /remote_cmd [autoware_msgs/RemoteCmd] 
- /steer_cmd [autoware_msgs/SteerCmd] 
- /twist_cmd [geometry_msgs/TwistStamped] 

## way_planner 

# waypoint_planner 

## velocity_set 

https://zhuanlan.zhihu.com/p/396386713  

该节点的主要工能是：接收safety_waypoints话题(astar_avoid输出路径)，结合实时点云信息，判断该路径上是否存在障碍物，如果存在障碍物，随机判断障碍物距离当前位置的距离，进而判断需要减速还是停车，进而修改当前位置到障碍物之间的路径点中的速度信息，避免突然减速；修改完成后发布修改后的路径finaobstacle_waypoint_pub_waypoints，以及障碍物所在的路径索引值obstacle_waypoint。 

Detect obstacles by using PointCloud. 

If no point cloud is available, means no obstacles, car keeps goging 

If use_crosswalk_detection is enabled, obstacles will be detected on crosswalks. Otherwise it will only detect obstacles on the lane. 

Publications: 
/detection_range [visualization_msgs/MarkerArray] 
/final_waypoints [autoware_msgs/Lane] 
/node_status [autoware_system_msgs/NodeStatus] 
/obstacle [visualization_msgs/Marker]: marker for obstacle 
/obstacle_waypoint [std_msgs/Int32]: -1 means no obstacle 
/stopline_waypoint [std_msgs/Int32]: -1 means no obstacle 

Subscriptions: 
- /config/velocity_set [autoware_config_msgs/ConfigVelocitySet] 
- /current_pose [geometry_msgs::PoseStamped]: pose of base_link 
- current_velocity[geometry_msgs::TwistStamped]: vehicle speed on twist.linear.x 
- /localizer_pose [geometry_msgs::PoseStamped]: pose of sensor 
- /points_no_ground [sensor_msgs::PointCloud2]: 
  - point.z > config->detection_height_top or < config->detection_height_bottom will be removed. 
  - sqrt(point.x^2+point.y^2)< remove_points_upto (from lidar_euclidean_cluster_detect_param node) will be removed. 
- /safety_waypoints [autoware_msgs/Lane]: from astar_avoid or astar_navi 
- /state/stopline_wpidx [std_msgs/Int32]: detect result from other nodes (waypoint ID) 
- /vector_map_info/area [vector_map_msgs/AreaArray] 
- /vector_map_info/cross_walk [vector_map_msgs/CrossWalkArray] 
- /vector_map_info/line [vector_map_msgs/LineArray] 
- /vector_map_info/point [vector_map_msgs/PointArray] 

Note: some information about VectorMap: [ADASMap](https://tools.tier4.jp/vector_map_builder/user_guide/)

For the note to function, current_pose, safety_waypoints are needed. 

## velocity_set_lanelet2 
Similar function like velocity_set, instead of Vector Map, this node uses Lanelet2 

Publications: 
- /detection_range [visualization_msgs/MarkerArray] 
- /final_waypoints [autoware_msgs/Lane] 
- /node_status [autoware_system_msgs/NodeStatus] 
- /obstacle [visualization_msgs/Marker] 
- /obstacle_waypoint [std_msgs/Int32] 
- /stopline_waypoint [std_msgs/Int32] 

Subscriptions 
- /config/velocity_set [autoware_config_msgs/ConfigVelocitySet] 
- /current_pose [geometry_msgs::PoseStamped]: 
- current_velocity[geometry_msgs::TwistStamped] 
- lanelet_map_bin[autoware_lanelet2_msgs::MapBin]: Lanelet2 binary format message 
- /localizer_pose [geometry_msgs::PoseStamped] 
- points_no_ground[sensor_msgs::PointCloud2] 
- safety_waypoints[autoware_msgs/Lane]: 
- /state/stopline_wpidx [std_msgs::Int32] 

## astar_avoid 

A* avoidance method, see README. 

Only runs if speed is lower than avoid_start_velocity 

necessary topics are current_pose, closest_waypoint, base_waypoints, closest_waypoint 

if enable_avoidance, then current_velocity and /semantics/costmap_generator/occupancy_grid are also needed. 

Publications: 
- /safety_waypoints [autoware_msgs/Lane] 

Subscriptions: 
- /base_waypoints [autoware_msgs/Lane] 
- /closest_waypoint [std_msgs/Int32] 
- /current_pose [geometry_msgs/PoseStamped] 
- /current_velocity [geometry_msgs/TwistStamped] 
- /obstacle_waypoint [std_msgs/Int32] 
- /semantics/costmap_generator/occupancy_grid [nav_msgs/OccupancyGrid] 

# freespace_planner 
## ASTAR_NAVI 

`astar_navi` is global path planner based on Hybrid A* search algorithm in `astar_search` package. This node executes planning at a constant cycle and publish `lane_waypoints_array`. 

Publications:  
- /lane_waypoints_array [autoware_msgs/LaneArray]  

Subscriptions: (all necessary)  
- /current_pose [geometry_msgs/PoseStamped] 
- /move_base_simple/goal [geometry_msgs/PoseStamped] 
- /semantics/costmap_generator/occupancy_grid [nav_msgs/OccupancyGrid] 

## wf_simulator 
使用接收到的车辆控制信号（v，ω）模拟理想的自身位置和速度 