# Message to specify positions for the NAO's motor joints.
#
# Each joint is uniquely identified by its index (See JointIndexes.msg)
#
# The three arrays in this message should have the same size, where the first item
# in indexes, corresponds to the first item in positions, etc.
std_msgs/Header header

float32 DEFAULT_HARDNESS = 0.7

uint8[] indexes  # See JointIndexes.msg (eg. JointIndexes::HEADYAW)
float32[] positions # radians
float32[] stiffnesses  # 0.0 - 1.0
