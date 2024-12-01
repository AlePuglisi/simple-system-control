## simple-system-control 

ROS 2 Humble | C++ | Gazebo Classic | ros2_control | effort_controllers | PID motor control

### Description
<img align="left" width="200" height="300" src="https://github.com/user-attachments/assets/27218d15-24fa-4380-8c5a-bda76b530df9"/>
This repository provides ros2 packages for the control of a simple 2 links - 1 joint system. <br/>
The simple model is supposed to emulate a fixed motor moving a simple cylindrical load. <br/>
Differently from any other ros2_control projects, here I implement a custom node for the control of joint position, based on a cascade controller. 

This project was born because of some difficulties in implementing transmissions in ros2_control, for simulating motor's gearbox (still not working, any advice is welcome!). <br/><br/>
Anyway, I found this valuable for anyone experimenting with ROS 2, Gazebo, ros2_control, URDF, PID tuning and anti-wind up experiments (for now Back calculation algorithm).

<br/>
<br/>
<br/>
 
### Packages functionalities: 
- **system_description**: Provide Rviz configuration and URDF of the model
- **system_bringup**: Just a bring-up node, storing all launcher
- **system_command_interface**: Map PS4 joypad commands or command line user commands into reference signals.
- **system_control**: Implement the control algorithm, and save results in a csv file for reference-tracking performance analysis through a Python script.
                   <br/> read data from ros2_control state interface and write effort commands by an effort_controller.

### Run the code: 

#### Setting up the packages

For [newcomers in ROS 2](https://docs.ros.org/en/humble/Installation.html), after setting up ros 2 (desktop install) and creating the workspace as in the wiki, just clone this repo in your src: 

```
git clone https://github.com/AlePuglisi/simple-system-control.git
```

be sure to have installed:

```
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control
```
(I have tested it in humble, I'm not sure about previous ros2 version compatibility)

build your ros2 workspace and source (follow one of the many the tutorial for adding in your bashrc the setup.bash and colcon argcomplete)...

#### Start controller simulation: 

```
# Terminal 1
ros2 launch system_bringup control_simulate.launch.py
```
will run the controller node, Rviz, Gazebo, joy node and set up the controllers. 

Then in another terminal: 
```
# Terminal 2
ros2 run system_command_interface system_command_interface
```
Show below all the joypad commands, or command line equivalent, with the terminal input to publish the command if no joypad is available:

```
# Terminal 3, for command line commands:
ros2 topic pub --once /string std_msgs/msg/String '{data: YOUR_COMMAND}'
```

Now you are ready to move the load in the desired angle position!

#### Analyze controller performance: 

During the simulation, several signals have been saved in ~/system_control/data_record.csv
To visualize it, in a free terminal change the directory to ~/system_control/script and run:

```
# Terminal 3, for command line commands:
python3 plot_result.py
```
The names of the plotted signals are intuitively chosen to understand what is showing up. 


### Conclusion:
For any question, curiosity, or problem, don't hesitate to open an  issue or contact me. 

Soon I will add to this README also additional information on the controller scheme and tuning principle. 











