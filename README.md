# Autoware Planning Supervisor

This repository provides ROS2 nodes and Python scripts for automating Autoware planning tasks.

## Environment

- Autoware AD Kit (commit `6290501`)
- Python 3.10.12

## Installation

```shell
$ ls
LICENSE.txt  README.md  autoware_planning_supervisor/  launch/  package.xml  setup.cfg  setup.py  ...
$ colcon build
Starting >>> autoware_planning_supervisor
Finished <<< autoware_planning_supervisor [0.55s]

Summary: 1 package finished [0.68s]
```

## Example

```shell
$ source install/setup.bash
$ ros2 run autoware_planning_supervisor supervisor --log_dir /autoware_data/log
[INFO] [launch]: All log files can be found below /home/xxx/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/pointcloud_container/glog_component' in container '/pointcloud_container'...
...
(Press Ctrl-C to finish)
$ ls /autoware_data/log
...  supervise-20XX-XX-XX-XX-XXX/  ...
```

- Planning demonstrations on Rviz2 are invoked.
- Planning configurations are automatically set by the `PlanningTask` object.
- If a task is done, another task will be invoked.

## License

This artifact is released under the Apache 2.0 license, see `LICENSE.txt`.
