# NomadZ Next Generation

Framework built on top of modern ROS2 architecture.

## Setup

The framework can be developed locally on Ubuntu 22.04, follow the relevant instructions below.

### Ubuntu

**These instructions have been tested on __Ubuntu 22.04__, which is the only officially supported Linux distro.**

#### System

1. Install ROS2 **Humble** by following [the official instructions for your OS](https://docs.ros.org/en/humble/Installation.html).

__NOTE__: installation via [Debian packages](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) is the recommended installation method. __Make sure to install both `ros-humble-desktop` and `ros-dev-tools`.__

2. Install other system level dependencies:
```shell
sudo apt-get install \
    ccache \
    clang \
    clang-format \
    cmake \
    curl \
    clang-tidy \
    git \
    git-lfs \
    ninja-build \
    python3-colcon-mixin \
    python3-pip
```

3. Install __conan__  **(at least version 2.2.2)** with pip:
```shell
pip install conan==2.2.2
```
If you get this warning:
> WARNING: The script conan is installed in '/home/user/.local/bin' which is not on PATH.

you will need to add `~/.local/bin` to your [`$PATH`](https://www.gnu.org/software/bash/manual/bash.html#index-PATH) with:
```
export PATH=~/.local/bin:$PATH
```
__NOTE__: this will only add it for your current shell session, to make this change permanent __(recommended)__, you need to add that line to your `.bashrc`:
```
echo 'export PATH=~/.local/bin:$PATH' >> ~/.bashrc
```

#### Workspace


1. Clone this repository:
```
git clone https://github.com/nomadz-ethz/nomadz-code-release.git
cd nomadz-ng
```

2. In case you haven't install the vcstool, you can install it using:
```
sudo apt install python3-vcstool
```

3. Setup the workspace using the provided setup script:
```
./tools/setup/setup-workspace.sh
```
## Compilation

### Host

1. Source the ROS setup script, i.e. `setup.bash`, in the ROS install directory:
```
source /opt/ros/humble/setup.bash
```
__NOTE__: if you added this line to your `.bashrc` during the installation of ROS 2, this step is not required (this is automatically the case in the Docker devcontainer setup).

2. Set the `COLCON_HOME` environment variable to the `./.colcon` directory of the workspace:
```
export COLCON_HOME=./.colcon
```
__NOTE__: this must be done for every new terminal before building the workspace

3. Build with colcon:
```
colcon build
```
This will build workspace in `Debug` mode by default. To build in `Release` mode, pass the `host-release` mixin:
```
colcon build --mixin host-release
```

## Getting Started

### Setting up Webots Simulation
We utilize Webots to simulate our robot soccer environment. Follow the [official website](https://cyberbotics.com) installation instructions to set it up.
### Starting the ROS Node
`nomadz_bringup` provides a collection of launch scripts for starting the software stack.
1. Make sure to always source the workspace setup script in the ROS install directory. E.g:
    ```shell
    source install/setup.bash
    ```
2. Launch the bringup node using `ros2 launch`:
    ```shell
    ros2 launch nomadz_bringup single_robot_sim_launch.py
    ```
__Note__: you can also start the simulation in isolation by launching `single_robot_sim_webots_launch.py` and `single_robot_sim_robot_launch.py` in different terminals. Similarly, you can start the full game using the `full_game_sim_launch.py` file.

By default, the robot is controlled by the `behavior_node`. For manual control, you can disable it inside the [`sim_robot_nodes_launch.py`](./src/nomadz_bringup/launch/sim_robot_nodes_launch.py) and enable the `manual_control_node` for [keyboard control](./src/nomadz_teleop/README.md)

## Development

### Formatting

To make sure that all source code merged into master is compliant with our style guidelines, this repository includes configuration files for all the formatting tools that we use, namely:
* `clang-format` version __14__ for C/C++
* `black` for Python
* `cmake-format` for CMakeLists.txt files

### `pre-commit` hooks

In order to enforce formatting rules on all the commits, we use `pre-commit`. To install the pre-commit hooks in your local copy:
1. Install `pre-commit`
```
pip install pre-commit
```
2. Run `pre-commit install` from the root of the repository:
```
pre-commit install
```

## Tools
### Debugging and Visualization
We primarily use [Foxglove Studio](https://foxglove.dev/download) for debugging and visualization, but you can use any tools you prefer (e.g. [rqt](https://docs.ros.org/en/foxy/Concepts/About-RQt.html), [rviz](https://github.com/ros2/rviz), [plotjuggler](https://github.com/facontidavide/PlotJuggler)).

__Note__: [ROS Foxglove bridge](https://docs.foxglove.dev/docs/connecting-to-data/ros-foxglove-bridge/) is required to interface with the Foxglove Studio.


### NomadZ Obama
NomadZ Obama is the deployment tool for our code, that allows for multirobot deployment and gives visual feedback for the robot status.
For more information, refer to the [NomadZ Obama README](./tools/nomadz_obama/README.md).


## License

This software is distributed under the MIT License.
