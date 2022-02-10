# autoware_connector 

## can_status_translator 

extract speed (twist) from CAN msg and publish 

Subscriptions: 
- can_info[autoware_can_msgs::CANInfo] 
- vehicle_status[autoware_msgs::VehicleStatus] 

Publications: 
- can_velocity[geometry_msgs::TwistStamped]: 
- linear_velocity_viz[std_msgs::Float32] 
- vehicle_status[autoware_msgs::VehicleStatus] 

## can_odometry:  
convert VehicleStatus to odom (position and velocity)  

Subscriptions: 
- vehicle_status[autoware_msgs::VehicleStatus] 

Publications: 
- /vehicle/odom[nav_msgs::Odometry] 

# ekf_localizer 

EKF estimates vehicle pose and velocity using 2D vehicle dynamics  

Subscrptions: 
- measured_pose_with_covariance (geometry_msgs/PoseWithCovarianceStamped): Input pose source with measurement covariance matrix, used when use_pose_with_covariance is true. 
- measured_twist_with_covariance (geometry_msgs/PoseWithCovarianceStamped):Input twist source with measurement covariance matrix, used when use_twist_with_covariance is true. 
- measured_pose (geometry_msgs/PoseStamped): Input pose source, used when use_pose_with_covariance is false. 
- measured_twist (geometry_msgs/TwistStamped): Input twist source, used when use_twist_with_covariance is false. 
- initialpose (geometry_msgs/PoseWithCovarianceStamped): Initial pose for EKF. The estimated pose is initialized with zeros at start. It is initialized with this message whenever published. 

Publications 
- ekf_pose (geometry_msgs/PoseStamped): Estimated pose. 
- ekf_pose_with_covariance (geometry_msgs/PoseWithCovarianceStamped): Estimated pose with covariance. 
- ekf_twist (geometry_msgs/TwistStamped): Estimated twist. 
- ekf_twist_with_covariance (geometry_msgs/TwistWithCovarianceStamped): Estimated twist with covariance. 

# gnss_localizer 

## fix2tfpose 

Convert GPS latitude/longitude/altitude to local x/y/z using Japanese rectangular coordinate system 

Publications:  
- gnss_pose[geometry_msgs::PoseStamped] 
- gnss_stat[std_msgs::Bool] 
 
Subscriptions:  
- fix[sensor_msgs::NavSatFix] 

## nmea2tfpose
Convert GPS nmea sentence to local x/y/z using Japanese rectangular coordinate system 

Publications: 
- gnss_pose[geometry_msgs::PoseStamped] 

Subscriptions: 
- nmea_sentence[nmea_msgs::Sentence]## image_processor 

# lidar_apollo_cnn_seg_detect 

# lidar_euclidean_cluster_detect 

# lidar_fake_perception 

# lidar_imm_ukf_pda_track 

# lidar_kf_contour_track 

# lidar_localizer 

##  approximate_ndt_mapping 

## icp_matching 

## ndt_mapping 

subscribe 
- points_raw: the first received PointCloud2 transformed to base_link frame will be used as map origin. 

subscribe
- /imu_raw: use imu->orientation, imu->linear_acceleration.x, not using imu->linear_acceleration.y, imu->linear_acceleration.z 

## ndt_mapping_tku 

## ndt_matching 

- use_local_transform: if true, 3D map data is in world frame, so need to convert localization into local frame (map) 
- publish /localizer_pose: pose from ndt matching, in lidar frame 
- publish /ndt_pose: pose for base_link, uses /tf_x, /tf_y, /tf_z, /tf_roll, /tf_yaw, /tf_pitch 

subscribe /imu_raw: use imu->orientation, imu->linear_acceleration.x, not using imu->linear_acceleration.y, imu->linear_acceleration.z 

subscribe /vehicle/odom: use odom->twist, not using odom->pose. 

some notes on source code: 

local_transform: world -> map 

predict_pose: predicted pose using previous pose and previous speed, such as linear predict using speed (from last 2 poses) 

predict_pose_for_ndt from either GNSS/ODOM/IMU, is only used to give initial position for NDT matching. 

current_pose == ndt_pose 

tf_btol: transform data from lidar to base_link, or transform frame base_link to frame lidar 

tf_ltob: transform data from base_link to lidar, or transform frame lidar to frame base_link 

Publications: 
- /estimate_twist [geometry_msgs/TwistStamped] 
- /estimated_vel [geometry_msgs/Vector3Stamped] 
- /estimated_vel_kmph [std_msgs/Float32] 
- /estimated_vel_mps [std_msgs/Float32] 
- /localizer_pose [geometry_msgs/PoseStamped] 
- /ndt_pose [geometry_msgs/PoseStamped] 
- /ndt_reliability [std_msgs/Float32] 
- /ndt_stat [autoware_msgs/NDTStat] 
- /node_status [autoware_system_msgs/NodeStatus] 
- /predict_pose [geometry_msgs/PoseStamped] 
- /predict_pose_imu [geometry_msgs/PoseStamped] 
- /predict_pose_imu_odom [geometry_msgs/PoseStamped] 
- /predict_pose_odom [geometry_msgs/PoseStamped] 
- /tf [tf2_msgs/TFMessage] 
- /time_ndt_matching [std_msgs/Float32] 

Subscriptions: 

- /config/ndt [autoware_config_msgs/ConfigNDT] 
- /filtered_points [sensor_msgs/PointCloud2] 
- /gnss_pose [geometry_msgs/PoseStamped] 
- /imu_raw [sensor_msgs/Imu] 
- /initialpose [geometry_msgs/PoseWithCovarianceStamped] 
- /points_map [sensor_msgs/PointCloud2] 
- /vehicle/odom [nav_msgs/Odometry] 

## ndt_matching_monitor 

See README 

Publications: 
- /initialpose [geometry_msgs/PoseWithCovarianceStamped] 
- /ndt_monitor/ndt_info_text [jsk_rviz_plugins/OverlayText] 
- /ndt_monitor/ndt_status [std_msgs/String] 

Subscriptions: 
- /gnss_pose [geometry_msgs/PoseStamped] 
- /initialpose [geometry_msgs/PoseWithCovarianceStamped] 
- /ndt_pose [geometry_msgs/PoseStamped] 
- /ndt_stat [autoware_msgs/NDTStat] 

## ndt_matching_tku 

## queue_counter 

Subscribe to raw point cloud and ndt mapped point cloud, and generate progress on terminal: (Processed/Input): (xxx/xxx) 

Subscriptions: 
- points_raw[sensor_msgs::PointCloud2] 
- points_raw[ndt_map::PointCloud2] 

# lidar_naive_l_shape_detect 

# lidar_point_pillars 

# lidar_shape_estimation 

# naive_motion_predict 

# ndt_cpu 

# ndt_gpu 

# ndt_tku 

# obj_db 

# pcl_omp_registration 

# pixel_cloud_fusion 

# points_downsampler 

# distance_filter 

# random_filter 

# ring_filter 

# voxel_grid_filter 

Filter point cloud using Voxel grid. 

Publications: 
- /filtered_points [sensor_msgs/PointCloud2] 
- /points_downsampler_info [points_downsampler/PointsDownsamplerInfo] 

Subscriptions: 

- /config/voxel_grid_filter [autoware_config_msgs/ConfigVoxelGridFilter] 
- /points_raw [sensor_msgs/PointCloud2] 

# points_preprocessor 

# pos_db 

# range_vision_fusion 

# road_occupancy_processor 

# roi_object_filter 

# trafficlight_recognizer 

# twist_generator 

# vel_pose_diff_checker 

# vision_beyond_track 

# vision_darknet_detect 

# vision_lane_detect 

# vision_segment_enet_detect 

# vision_ssd_detect 