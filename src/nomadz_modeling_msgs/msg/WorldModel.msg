# The WorldModel summarize the all the local information the robot knows about the world.
# So information acquired from the team communication are not included in this message.
std_msgs/Header header

# Contain a copy of the ball model
nomadz_modeling_msgs/BallModel ball_model

# Contain a copy of the robot pose
nomadz_modeling_msgs/RobotPose robot_pose

# A list of the ids and the poses of the teammates and the opponents in world coordinates
geometry_msgs/Pose2D[] teammate_poses

geometry_msgs/Pose2D[] opponent_poses
