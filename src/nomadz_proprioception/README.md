# nomadz_proprioception
This package provides a ROS node for all proprioception related topics done on the robots.

Proprioception related stuff includes:
- arm contact estimation
- fall down state detector
- foot support estimator
- fsr calibrator (fsr := force sensor resistor)

All of those modules will run in the proprioception node.

## The proprioception Node

The proprioception node is subscribed to following topics
- imu_sub__
- joint_data_sub_
- fsr_sub_
- sync_

and it publishes:
- foot_support_publisher_
- fall_down_state_publisher_
- orientation_estimation_publisher
- robot_model_publisher_
- arm_contact_publisher_
<!-- - transform_broadcaster_
- static_transform_broadcaster_  -->

All of this messages are sent after their respective update function has been called. All except of the updateArmContactEstimate are being called in the onSensorDataReceived function. While the updateArmContactEstimate is called right when a requestedrobotmodel msg has been received.
