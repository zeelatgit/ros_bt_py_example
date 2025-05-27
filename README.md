# ros_bt_py_example
This repository has an experimental package to investigate and try out different functionalities of the framework 'ros_bt_py'.

## Dependencies
This package requires ros_bt_py, which is already present as a submodule in this repo. To clone the contents of the repo along with the submodule use this command:

```bash
git clone --recurse-submodules 
```


## Battery Simulator Node
Run the battery simulator using:
```bash
ros2 run example_package battery_simulator
```

The battery simulator always intialises with a 100%
The battery keeps dropping every second by 1 %. There is 10% chance that the battery may drop by 2-5% every second as well.

This node offers 3 Trigger services (empty services, can be called without any input):

- /start_charging : Charges the battery. Battery level stops dropping and starts increasing instead. Increases by 5% every second.
- /stop_charging : Stops charging the battery. Battery goes into nomral operation and the battery level starts dropping again.
- /reset_battery : Resets the battery to a 100% and stops charging if it is on.

* The battery state can be monitored on the topic: /battery_level

* To only monitor the batter percentage: /battery_percent

## Action Server

This a ROS2 action server. It accepts string commands and can trigger battery charging accordingly. To run the server:
```bash
ros2 run example_package action_server
```

Apart from that, the node also publishes a location (either 'home' or 'random_place') on the topic : /location

The server can handle 3 actions:

- execute_dummy_action : Prints an acknowledgement message and updates the location to 'random_place'. Retruns Success.

- go_to_home_n_charge : Prints an acknowledegment message and updates location to 'home'. Also, calls the /start_charging service. Returns Success.

- go_to_home : Prints an acknowledegment message and updates location to 'home'. Returns Success.

## Working with ros_bt_py

We can execute a behavior tree with ros_bt_py to interact with our battery_simulator and action server. To get started:

- Run the action server
- Run the battery simulator
- Launch the ros_bt_py WebGUI:
```bash
ros2 launch ros_bt_py ros_bt_py.launch.py
```
- Load the example package with the package loader. Type 'example_package.bt_nodes' and then click on 'Load Package'.

  *NOTE: You don't necessarily need this step for the existing behavior tree. All this does is, it loads the custom nodes, which resides in the example_package, to the webGUI. Your custom       
  messages, services and actions should automatically be available in the GUI, after building your workspace.*
  
- Now upload 'tree (7).yaml' available in the directory 'behavior_trees'
- You can now start the tree. To get a continuous process, you can select the 'Tick Periodically' option.
- To execute an action publish a command on the topic '/command_topic':
```bash
ros2 topic pub /command_topic std_msgs/msg/String "data: 'execute_dummy_action'" --once
```
or

```bash
ros2 topic pub /command_topic std_msgs/msg/String "data: 'go_to_home_n_charge'" --once
```
or 

```bash
ros2 topic pub /command_topic std_msgs/msg/String "data: 'go_to_home'" --once
```







