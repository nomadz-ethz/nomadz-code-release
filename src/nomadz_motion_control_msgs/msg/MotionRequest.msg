std_msgs/Header header

# Motion type
# SPECIAL_ACTION: Use only special action request
# WALK: Use only walk_request
# IN_WALK_KICK: Use both walk_request's pattern mode and kick_request
# KICK: Use only kick_request
uint8 motion_type

SpecialActionRequest special_action_request
WalkRequest walk_request
KickRequest kick_request
HeadMotionRequest head_motion_request
