#  autoware_bag_tools 

- change_frame_id: change frame_id from topics 
- replace_msg_time_with_hdr: replace timestamp in a message with the header message 
- nmea2kml: extract GPS data from rosbag file(s) into .kml and .csv files (need topic /nmea_sentence) 

#  autoware_camera_lidar_calibrator 
- cameracalibrator: camera intrinsic calibration 
- camera_lidar_calibration: camera-LiDAR extrinsic calibration 

#   calibration_publisher
publishes the camera intrinsics, extrinsics and registers the TF between the camera and LiDAR sensors. 

#  autoware_launcher 

I think this is an early version of runtime_manager, don’t know how to use and the purpose of it. 

#  autoware_launcher_rviz 

A RViz panel plugin, connect to server localhost at port 33136 

#  data_preprocessor 

- get_Image: save image from ROS topic into disk (jpg) 
- get_Depth: get depth image and save to disk from 2D image and 3D point cloud topic. 
- get_PCD: save point cloud from ROS topic into disk (PCD ASCII) 

#  graph_tools 

- yaml2dot: convert YAML into DOT 
- kitti_box_publisher 

publish KITTI bounding box based on PointCloud topic seq (need a config xml file as input, specifying KITTI data location) 

#  kitti_launch 

#  kitti_player 

#  lanelet_aisan_converter 

#  log_tools 

#  map_tf_generator 

Subscribe to map topic, and calculate the average (mean) of the point cloud map. The negative of mean is used to publish as the tf from map to world. 

Subscriptions: 
- /points_map [sensor_msgs/PointCloud2] 

#  map_tools 

#  marker_downsampler 

#  mqtt_socket 

#  multi_lidar_calibrator 

#  oculus_socket 

#  pc2_downsampler 

#  rosbag_controller 

#  runtime_manager 

setup: baselink to localizer: means transform needed to move from baselink to localizer 

#  sound_player 

Play different sounds such as red/green light, start, stop etc. I don’t think this package works. There are obvious bugs. 

#  sync 

#  tablet_socket 

# # twist2odom 

#  udon_socket 

#  vehicle_engage_panel 

#  vehicle_socket 

#  vehicle_sender 

Receive autoware_msgs::VehicleCmd from /vehicle_cmd 

Listen on port 10001 for socket connection, and send back vehicle command data in a string, separated by ‘,’ 

Subscriptions: 
- /vehicle_cmd [autoware_msgs/VehicleCmd] 
#  vehicle_receiver 

Listen on port 10000 for socket connection, receive can data and pub on /can_info and /mode_info 

Publications: 
- /can_info [autoware_can_msgs/CANInfo] 
- /mode_info [tablet_socket_msgs/mode_info] 