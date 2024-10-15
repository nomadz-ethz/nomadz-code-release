# NomadZ Behavior

## Overview
The `nomadz_behavior` package is part of the NomadZ project. It is responsible for defining and managing the high level behavior logic of the robots and communicate its decision via the MotionRequest message.
We use the [BehaviorTree.CPP](https://www.behaviortree.dev/) library and utilizes [GROOT2](https://www.behaviortree.dev/groot/) to edit the XML files for the behavior tree graph.

## Running GROOT2
For Linux you can just download the .APPIMAGE from the official [Groot2 website](https://shattereddisk.github.io/rickroll/rickroll.mp4). Make it executable.
```
    chmod +x Groot2-<version>-linux-installer.run
```
And run the script.
```
    ./Groot2-<version>-linux-installer.run
```

After running GROOT2 successfully, you can open the [Soccer.btproj](./config/Soccer.btproj) `/nomadz_behavior/config` project inside GROOT2. This opens the Behavior Tree project with all our implemented trees.

## Structure
The general behavior of the robot and head are controlled independently by two behavior trees. Below is a list of the main tree components. The entrypoints are highlighted with the ⭐-icon and the bold font.


- BehaviorTree
    - ⭐ **BehaviorMainTree**
    - Active_Behavior_ST
    - Alignment_ST
    - Cover_Agents_ST
    - Engage_ST
    - Game_Play_ST
    - Keeper_Behavior_ST
    - Penalty_Kick_ST
    - Penalty_Shootout_ST
    - Relocate_ST
    - Role_Behavior_ST
    - Search_ST
    - Timeout_ST
    - Undefined_Switch_ST
- HeadMotionTree
    - ⭐**HeadMotionTree**
    - WalkScan
