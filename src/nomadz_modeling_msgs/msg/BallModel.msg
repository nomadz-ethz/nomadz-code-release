std_msgs/Header header
# Estimated ball position and velocity in base frame
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity

builtin_interfaces/Time last_valid_time_stamp
# Ball is considered valid if it was detected within the BALL_NOT_SEEN_TIMEOUT duration and lost if it was not detected within the BALL_LOST_TIMEOUT duration
bool valid
bool lost
