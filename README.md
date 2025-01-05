# rt1_assignment2_1

## Summary

This package demonstrates the implementation of a task using an action client in ROS. The goal is to manage robot navigation to a target while ensuring non-blocking behavior. The solution is developed across two nodes and a launch file, as described below:

### Nodes Implemented:
*Action Client Node and subscriber-publisher:*

- Sends target coordinates (x, y) to the action server.
- Provides the ability to cancel the target during execution.
- Monitors feedback and status from the action server to determine when the target is reached.
- Publishes robot position and velocity as a custom message (x, y, vel_x, vel_z) based on data from the /odom topic.

*Service Node:*

- Responds to requests with the coordinates of the last target sent by the user.

### Launch File:

Starts all the nodes and initializes the simulation environment.

## Installation

First, make sure you have assignment_2_2024 in your workspace. 

You can clone it from github with this link :

https://github.com/CarmineD8/assignment_2_2024


Then, clone the rt1_assignment2_1 repository into your ros workspace and compile with:

    catkin_make

Then, source your workspace with:

    source devel/setup.bash

## Launch the Simulation

To launch the whole simulation environment and the two nodes, use the following command:

```sh
roslaunch rt1_assignment2_1 all_assignment.launch
```

### Interact with the Robot

Once the nodes launched, you have three choices with the robot_target node

1. Enter a target coordinate
    ```
    set x y
    ```
    for example
    ```
    set 3.2 2.5
    ```
2. Cancel a target coordinate sent and stop the robot
    ```sh
    cancel
    ```
3. End the node
    ```
    end
    ```

You can also in another terminal call the service from the node *last_target_service* to get the last target position of the robot:
```sh
rosservice call /get_last_target
```

## Nodes

### robot_target

This node is separated of two parts. One is responsible of using the *PlanningAction* from the package assignment_2_2024 to send the coordinate to the action server *reaching_goal*. It works with a thread where we can at any time send a target or cancel the actual one. The other part is a subsriber-publisher which subscribe to the topic /odom and publish in the topic /odom_simplified with the custom message *Odom_simplified*

#### Subscriber

- Topic : `/odom`  Receives the robot's odometry data.
- Type : `nav_msgs/Odometry`

#### Publisher

- Topic : `/odom_simplified` Publishes velocity commands to control the robot.
- Type : `rt1_assignment2_1/Odom_simplified`



### last_target_service

This service node subscribe to the action goal topic */reaching_goal/goal* and store it in a global variable last_target. When the service `/get_last_target` is called, the service return the data of the variable stored.

#### Services

- `/get_last_target`: Provides the last target position the robot was commanded to move to.
- Custom_service : `Target`
- Answer type : `geometry_msgs/PoseStamped`

#### Subscriber

- Topic : `/reaching_goal/goal`  Receives the goal coordinate
- Type : `assignment_2_2024/PlanningActionGoal`

## Launch Files

### all_assignment.launch

This launch file :

- Launch assignment1.launch from the package assignment_2_2024
- Run the node robot_target and print the log on the console to see the instruction to send the goal coordinate and cancel 
- Run the node last_target_service

## Author

- Paul Pham Dang - 7899827
- I used the docker image with ROS Noetic
