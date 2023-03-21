
# Robot Configuration
The [Delta Robot](https://en.wikipedia.org/wiki/Delta_robot) taken into account is depicted as in the figure. Its four geometric parameters needed to perform the kinematics of the robot are set [here](https://github.com/danielemorra98/delta_robot_ROS2/blob/master/delta_robot/delta_robot_node.py#L14). If you change them, remember to build again the package to make the modification effective.

![image](https://user-images.githubusercontent.com/48955695/226612103-5698517e-5829-4394-ba87-b702d82a4c36.png)

The base_link and end_effector reference systems follow the configuration depicted in the figure below. The XY-plane containing the base platform is at `z=0`.

![image](https://user-images.githubusercontent.com/48955695/226613900-ad92f0d0-489a-4e7d-a893-d98d65f7d505.png)

# ROS2 functionalities
The executable would create:
1. A ROS2 Publisher of [JointState msg](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html) onto _delta_robot/desired_joint_state_ topic
2. A ROS2 Subscriber of [Vector3 msg](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Vector3.html) onto _delta_robot/desired_position_ topic
3. A ROS2 Subscriber of [JointState msg](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html) onto _delta_robot/joint_states_ topic

When the executable is run, it starts listening to the _delta_robot/desired_position_ topic which stands for the cartesian position of the end-effector the user could set. If an inputs arrives on that topic the inverse kinematics is performed and subsequentally, the corresponding angles of the motorized joints are published to  the _delta_robot/desired_joint_state_ topic.
The Subscriber to _delta_robot/joint_states_ listens to the joints state feedback, e.g. coming from a simulation (useful when the [plotting is activated](https://github.com/danielemorra98/delta_robot_ROS2/blob/a488cfaa7a4e273fdd6a7d6ebb94e693e0fcf6ac/delta_robot/delta_robot_node.py#L286))

# Run
To run the executable:
```
cd ros2_ws
source install/setup.bash
ros2 run delta_robot deltaKinematics 
```

In order to send the Cartesian position of the end-effector:
```
ros2 topic pub /delta_robot/desired_position geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: -0.40}"
```

# Installation
Clone the repository inside your ROS2 workspace and build the package:
```
cd ros2_ws/src
git clone https://github.com/danielemorra98/delta_robot_ROS2.git
colcon build --packages-select delta_robot
```
