# ros-gazebo
Experimentation with ROS and gazebo

## Getting control over the 3-dof RRR robotic manipulator

### Structure of the code
As recommended by the ROS community, the whole ros code for the robot/manipulator are contained in three different packages.
#### `tresgdl_description`
Basically just the xacro file defining the robot.
#### `tresgdl_gazebo`
Just the launch files for the moment. But could include special world files to spawn the model into.
#### `tresgdl_control`
Joint controllers, but also own controllers for control at higher level (moving to different poses, etc).

### Elements needed in the urdf/xacro file
  * Apart from the pure basics needed (links, joints) the file must contain `<transmission>` tags defining each actuated joint.
  * There must also be a `<gazebo>` tag telling gazebo to load the ros control plugin:
    `<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/tresgdl</robotNamespace> 
  </plugin>
   </gazebo>`
### Configuration file
This is found in `tresgdl_control/config/`. The file tells which controllers to load for each actuated joint. Values for the controller (PID gains) are set. Be careful that the names of the joints matches the urdf.

### Launch files
The main launch file is found in `tresgdl_gazebo/launch`. This launch file includes the launch file in `tresgdl_description` to load the robot description into the parameter server, and to start an empty world. 

The second step is to start the `robot_state_publisher` node and the `joint_state_publisher`node.

Finally the file includes the launch file for the control, in `tresgdl_control/launch`. The latter launch file loads the configuration file into the parameter server, and starts the ros control node. 

### Basic control 
A simple controller for the overall movement of the arm is contained in `tresgdl_control/sripts/move_robot.py`. Starting up a node running this controller will give some rudimentary way of interacting with the arm through the topics `/movecommand` and `/move_to_point`. The first topic takes an integer value, and will move the arm to either of a (small) pre-defined states. The second topic takes a `Vector3`, and is meant to take a desired position for the tool center point and move the robot there. This functionality is not yet implemented. 

### Activities
	1. Test the package. Run `rosrun tresgdl_control move_robot.py` to start the node that controls the movement. Publish some different values to `\movecommand` (0, 1 or 2).
	2. Implement a way to smoothly move the arm between states. Your professor will give suggestions.
	3. Implement the inverse-kinematics solution for the arm, and complete the code to enable `/move_to_point`
