# nomadz_bringup

This package groups all the launch files for the NomadZ ROS 2 software stack, following the ubiquitous set of conventions defined [here](https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/#What_your_package_organization_will_look_like).

## List of available launch files

Here is a (potentially not up-to-date) list of the launch files included in this package:

* **main_launch.py**
This launch file brings up the whole NomadZ software stack. Arguments:
  - _namespace_ : the namespace that should be inherited by all the nodes (default: empty string)
