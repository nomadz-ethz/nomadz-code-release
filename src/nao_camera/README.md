# nao_camera

This package provides a ROS node to interface with a physical or simulated NAO V6 camera device and publish the captured images over ROS topics. The package includes the following build targets:
* `nao_camera`: library which includes the [`NaoCameraNode`](include/nao_camera/node.hpp) ROS node.
* `nao_dual_camera`: an [executable](src/main.cpp) which creates a single-threaded executor spinning two instances of `NaoCameraNode`, one for the lower camera and one for the upper camera.

`NaoCameraNode` takes as one of its input arguments a [`BaseCameraDevice`](include/nao_camera/base_device.hpp) object. `BaseCameraDevice` is an abstract class defining a high-level interface for a camera device. This interface is implemented by two classes:
 * [`SimulatedCameraDevice`](include/nao_camera/sim_device.hpp): connects to an image TCP server and provided by the `WebotsLoLAController`
 * [`NaoCameraDevice`](include/nao_camera/phys_device.hpp): connects to and configures using `v4l2` to a physical camera device on the NAO V6.


### Simulated camera

When the package is compiled for the `host`, only `SimulatedCameraDevice` is built and used to instantiate `NaoCameraNode`.

To use it:

1. Start the `nao_robocup.wbt` scenario from [`WebotsLoLAController`](https://github.com/Bembelbots/WebotsLoLaController) in Webots.

2. Open a terminal, source the setup script of the NomadZ `colcon` workspace and start an instance of [`nao_lola_client`](https://github.com/nomadz-ethz/nomadz_nao_lola):
```
source install/setup.bash
ros2 run nao_lola_client nao_lola_client
```
3. When the client has connected, open a terminal, source the setup script of the `nomadz-ng` workspace and launch the camera driver:
```
source install/setup.bash
ros2 launch nao_camera sim_camera_launch.py
```
