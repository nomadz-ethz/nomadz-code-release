# Example Nao Lola Controller

This package shows how to write a node which uses nao_lola to read and write joint data to the simulation (or robot).

Essentially, the simulation and the robot work the same way in that they expose a socket which receives and publishes joint data. nao_lola is the ros2 node which connects to this socket and exposes ros2 messages which allow you to read and write data to the sim/robot. This node demonstrates how you create a simple controller which reads joint data and publishes joint data to nao_lola.

## How to use.
1. Install webots (https://cyberbotics.com/doc/guide/installation-procedure)
2. Install WebotsLolaController using `git clone git@github.com:nomadz-ethz/WebotsLoLaController.git` (Note, this does not need to be in the workspace. you can clone it into your preferred git folder)
3. Build this package using `./build-host.sh Release example_nao_lola_control`. Note that it depends on `nao_lola_command_msgs` and `nao_lola_sensor_msgs`
4. Source your environment
5. Launch webots
6. In a terminal, source `install/setup.bash` and launch the `ros2 launch nao_lola_client nao_lola_client_launch.py`
7. In a terminal, source `install/setup.bash` and launch the `ros2 run example_webots_control example_webots_control`
