## simple-system-control 

ROS 2 Humble | C++ | Gazebo Classic | ros2_control | effort_controllers | PID motor control

- ### DESCRIPTION
<img align="left" width="200" height="300" src="https://github.com/user-attachments/assets/27218d15-24fa-4380-8c5a-bda76b530df9"/>
This repository provides ros2 packages for the control of a simple 2 links - 1 joint system. <br/>
The simple model is supposed to emulate a fixed motor moving a simple cylindrical load. <br/>
Differently from any other ros2_control projects, here I implement a custom node for the control of joint position, based on a cascade controller. 

This project was born because of some difficulties in implementing transmissions in ros2_control, for simulating a motor's gearbox (still not working, any advice is welcome!). <br/><br/>
Anyway, I found this valuable for anyone experimenting with ROS 2, Gazebo, ros2_control, URDF, PID tuning and anti-wind up experiments (for now Back calculation algorithm).

To better understand the control scheme and the tuning procedure, you can refer to MATLAB and Simulink files. 
Soon I will update the README with additional information about the control scheme, for those who don't have a MathWorks License ;). 
<br/>
 
- ### PACKAGES IN BRIEF
  - **system_description**: Provide Rviz configuration and URDF of the model
  - **system_bringup**: Just a bring-up node, storing all launcher
  - **system_command_interface**: Map PS4 joypad commands or command line user commands into reference signals.
  - **system_control**: Implement the control algorithm, and save results in a ".csv" file for reference-tracking performance analysis through a Python script.
                   <br/> read data from ros2_control state interface and write effort commands by an effort_controller.

- ### RUN THE CODE
> [!NOTE]  
> I have tested the code with humble, I'm not sure about older ros2 version compatibility

  - #### Setting up the packages

    For [newcomers in ROS 2](https://docs.ros.org/en/humble/Installation.html), after setting up ROS 2 (desktop install) and creating the workspace as in the wiki, just clone this repo in your src: 
    
    ```
    git clone https://github.com/AlePuglisi/simple-system-control.git
    ```
    
    be sure to have installed:
    
    ```
    sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control
    ```

    build your ros2 workspace and source (follow one of the many the tutorial for adding in your bashrc the setup.bash and colcon argcomplete)...
  
  - #### Start controller simulation: 
  
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

  - #### Analyze controller performance: 
  
  During the simulation, several signals have been saved in ~/system_control/data_record.csv
  To visualize it, in a free terminal change the directory to ~/system_control/script and run:
  
  ```
  python3 plot_result.py
  ```
  The names of the plotted signals are intuitively chosen to understand what it is showing up. 
  > [!NOTE]  
  > This Python script uses pandas and matplotlib, be sure to have those libraries installed. 


- ### CONCLUSIONS

I believe that this code could be of help to anyone struggling with Laplace domain controller implementation. It is always hard to go from theory to practice. 
Also, experimenting with anti-wind-up is very useful to understand the effect of actuator limitations, try to play with the anti-wind-up term, and the motor limits, to see what I'm talking about. 
Changing gains, removing gravity compensation terms, or increasing load length or mass, show the power of this cascade control scheme. 

As a final note, this controller could be implemented using a custom controller in ros2_control, to make it more real-time reliable and integrated with ros2_control itself. 
It is not an easy task, but for sure an improvement. 

For any question, curiosity, or problem, don't hesitate to open an  issue or contact me. 












