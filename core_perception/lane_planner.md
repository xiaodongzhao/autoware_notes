## lane_navi 

Receive commands from route_cmd, align with /vector_map_info, then publish routes at /lane_waypoints_array 

route_cmd is a list of waypoints with only lat and lon in it. 

Publications: 
- /lane_waypoints_array [autoware_msgs/LaneArray] 

Subscriptions: 
- /route_cmd [tablet_socket_msgs/route_cmd] 
- /vector_map_info/lane [vector_map_msgs/LaneArray] 
- /vector_map_info/node [vector_map_msgs/NodeArray] 
- /vector_map_info/point [vector_map_msgs/PointArray] 

## lane_rule_lanelet2 

lane_rule_lanelet2 (using Lanelet2) detects stoplines that intersect with waypoints, and changes velocity of waypoints so that vehicle can stop before stopline when traffic_light is red. It actually publishes both green_waypoints and red_waypoints, and lane_stop will choose which waypoint to use according to the recognized traffic_light color 

Publications: 
- /green_waypoints_array [autoware_msgs/LaneArray]: waypoints that are meant to be used when traffic light is green 
- /red_waypoints_array [autoware_msgs/LaneArray]: waypoints that are meant to be used when traffic light is red 
- /traffic_waypoints_array [autoware_msgs/LaneArray]: waypoints with no change 

Subscriptions: 
- /config/lane_rule [autoware_config_msgs/ConfigLaneRule]: topics to update parameters at runtime. Note that stopline_search_radius member is not used in this node. 
- /lane_waypoints_array [autoware_msgs/LaneArray]: global waypoints 
- /lanelet_map_bin [lanelet_msgs/MapBin]: binary data of lanelet2 map 

## lane_rule 

lane_rule (using VectorMap): similar functions like lane_rule_lanelet2, except using a different map format VectorMap. 

Publications: 
- /green_waypoints_array [autoware_msgs/LaneArray] 
- /red_waypoints_array [autoware_msgs/LaneArray] 
- /traffic_waypoints_array [autoware_msgs/LaneArray] 

Subscriptions: 
- /config/lane_rule [autoware_config_msgs/ConfigLaneRule] 
- /lane_waypoints_array [autoware_msgs/LaneArray] 
- /vector_map_info/dtlane [vector_map_msgs/DTLaneArray] 
- /vector_map_info/lane [vector_map_msgs/LaneArray] 
- /vector_map_info/node [vector_map_msgs/NodeArray] 
- /vector_map_info/point [vector_map_msgs/PointArray] 
- /vector_map_info/stop_line [vector_map_msgs/StopLineArray] 

## lane_select: 
Receive command from /decision_maker/state and multiple routes 

Calculate the nearest point from the current position for all routes 

Route for the current lane with the nearest point 

Detects left and right routes to current route 

The lane change flag of the nearest point on the current route is retained as the lane change flag for that route. 

Find the nearest point with right turn or left turn flag, generate a route interpolating Hermite Curve between the point and the target point of the lane where the lane change is to be performed, and combine the route with the route after the target point where the lane change is scheduled and define route as lane after change 

If lane change is not performed, publish the current route, the nearest point to it, and the lane change flag 

When changing lanes, publish the lane change route, the nearest point to it, and the lane change flag. 

Note: 

In order to change lanes, it is necessary to read multiple path files in ver3 format. (See waypoint_maker package) 

current_pose, traffic_waypoints_array, current_velocity are needed for this node to work 

Publications: 
- /base_waypoints [autoware_msgs/Lane]: current lane 
- /change_flag [std_msgs/Int32]: one of straight=0, right=1, left=2 or unknown=-1 
- /closest_waypoint [std_msgs/Int32]: ID of closest waypoint from the current vehicle position 
- /current_lane_id [std_msgs/Int32]: ID of current lane 
- /lane_select_marker [visualization_msgs/MarkerArray]: 
- /vehicle_location [autoware_msgs/VehicleLocation]: current waypoint and lane the vehicle is on 

Subscriptions: 
- current_pose [geometry_msgs/PoseStamped]: 
- traffic_waypoints_array [waypoint_follower/LaneArray]: 
- state [std_msgs/String]: 
- current_velocity [geometry_msgs/TwistStamped]: 
- config/lane_select [runtime_manager/ConfigLaneSelect]: 
- /decision_maker/state [std_msgs/String]: one of `LANE_CHANGE, *UNKNOWN, indicate change lane or not 

## lane_stop 
Receive TrafficLight status from either light_color or light_color_managed (based on /config/lane_stop) 

if is RED, publish red_waypoints_array to traffic_waypoints_array 

if is GREEN, publish green_waypoints_array to traffic_waypoints_array 

Publications: 
- /traffic_waypoints_array [autoware_msgs/LaneArray] 

Subscriptions: 
- /config/lane_stop [autoware_config_msgs/ConfigLaneStop] 
- /green_waypoints_array [autoware_msgs/LaneArray] 
- /light_color [autoware_msgs/TrafficLight] 
- /light_color_managed [autoware_msgs/TrafficLight] 
- /red_waypoints_array [autoware_msgs/LaneArray] 

 