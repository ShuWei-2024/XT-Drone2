# ROS 2 equivalent of livox_ros_driver/CustomMsg.msg.
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
astra_fast_livo2_msgs/LivoxCustomPoint[] points
