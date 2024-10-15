# NomadZ logging

This package gives some basic functionality for recording ros topics and for monitoring cpu and ram usage using an additional ros node.

## Recording a vision bag

Bag files recorded with the `record_vision_bag_launch.py` can be reprocessed with the bag processor in `nomadz_image_processing`. The topics that will be recorded are specified in [this config file](./config/vision_topics.yaml). By default, the bag will saved to `bags/vision/nao_<TIMESTAMP>` relative the current working directory. An alternative bag path can be specified by passing the `bag_path` launch argument i.e. `ros2 launch nomadz_logging record_vision_bag.launch bag_path:=<YOUR_BAG_PATH>`.

### Recording data from a simulated robot

1. Start the Webots simulation with at least one robot with cameras enabled.
2. Open a new terminal and run:
```
ros2 launch nomadz_logging record_vision_bag_launch.py
```

### Recording data from a physical robot

1. Turn on a robot, deploy, and then connect your PC to the robot with an Ethernet cable.
2. Open a terminal and set the ROS_DOMAIN_ID environment variable to the last two digits of the robot IP address. For instance, if you are connected to robot whose IP address ends in 180:
```
export ROS_DOMAIN_ID=80
```
3. Now launch the bag recorder:
```
ros2 launch nomadz_logging record_vision_bag_launch.py
```
