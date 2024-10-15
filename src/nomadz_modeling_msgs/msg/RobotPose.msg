std_msgs/Header header

# 2D pose of a robot in world coordinate
geometry_msgs/Pose2D pose
builtin_interfaces/Time last_valid_time_stamp
# own pose is considered valid if it was detected within the POSE_VALID_TIMEOUT duration and lost if it was not detected within the POSE_LOST_TIMEOUT duration
bool valid
bool lost
