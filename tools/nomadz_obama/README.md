### NomadZ Obama
NomadZ Obama will be our deployment tool for Robocup
it offers basic functionalities such as:
* battery information
* connection status
* robot setup
    * robot role selection
    * robot id selection
    * wlan config selection
* group actions
    * deployment
    * restart nomadz-ng
    * reboot
    * shutdown
    * download logs
to start NomadZ Obama

1. make sure you have NiceGui installed

__Note__: [optional] Create a virtualenv of your preference and make sure you re running python >= 3.10
    eg. ``` conda create -n nomadz_obama python=3.10```
```
pip install nicegui returns
```
2. run the `main.py` file which is located in ```nomadz-ng/tools/nomadz_obama/```

```
python tools/nomadz_obama/main.py
```
Assign the robots an ID by dragging them to the "field". The ids are assigned from left to right 1-7.

To perform the other actions like assigning the wlan config you first need to select the robots using the checkbox and then change the field in the drop-down selection.

Once selected you can then perform the actions using the buttons on the right side of the GUI.
