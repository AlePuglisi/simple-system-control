import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def plot_joint_states(file_path):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(file_path)
    
    # Extract and normalize the 'time' column
    time = df['time'].tolist()
    time = [t - time[0] for t in time]  # Normalize time by subtracting the first element
    
    # Extract joint states and references, ensuring correct parsing
    position = df['current_position'].tolist()
    position_references_filtered = df['reference_position_filtered'].tolist()
    position_references = df['reference_position'].tolist()
    position_error = df['position_error'].tolist()
    velocity = df['current_velocity'].tolist()
    velocity_references = df['reference_velocity'].tolist()
    velocity_feed_forward = df['feed_forward_velocity'].tolist()
    velocity_error = df['velocity_error'].tolist()
    integral = df['integral_action'].tolist()
    proportional = df['proportional_action'].tolist()
    control_torque = df['control_torque'].tolist()
    
    
    plt.figure('Joint State tracking')

    plt.subplot(2,4,1)
    plt.plot(time, position,'b', label='joint position')
    plt.plot(time, position_references, 'r--', label=' joint position reference')
    plt.plot(time, position_references_filtered, 'g--', label=' joint position reference filtered')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('Joint angle [rad]')
    plt.title('Motor Joint Position States and References')
    plt.grid(True)

    plt.subplot(2,4,2)
    plt.plot(time, velocity,'b', label='joint velocity')
    plt.plot(time, velocity_references, 'r--', label=' joint velocity reference')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('Joint velocity [rad/s]')
    plt.title('Motor Joint Velocity States and References')
    plt.grid(True)

    plt.subplot(2,4,3)
    plt.plot(time, velocity_feed_forward, 'g', label=' joint velocity feed forward')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('Joint velocity feed forward [rad/s]')
    plt.title('Motor Joint Velocity Feed Forward')
    plt.grid(True)

    plt.subplot(2,4,4)
    plt.plot(time,proportional,'b', label='proportional action')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('proportional action torque [Nm]')
    plt.title('Motor Joint proporitonal torque')
    plt.grid(True)

    plt.subplot(2,4,5)
    plt.plot(time,position_error, 'r', label='joint position error')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('position error [rad]')
    plt.title('Motor Joint Position error')
    plt.grid(True)

    plt.subplot(2,4,6)
    plt.plot(time,velocity_error,'r', label='joint velocity error')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('velocity error [rad/s]')
    plt.title('Motor Joint Velocity error')
    plt.grid(True)

    plt.subplot(2,4,7)
    plt.plot(time,control_torque,'b', label='joint torque')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('torque [Nm]')
    plt.title('Motor Joint torque')
    plt.grid(True)

    plt.subplot(2,4,8)
    plt.plot(time,integral,'b', label='integral action')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.ylabel('integral action torque [Nm]')
    plt.title('Motor Joint integral torque')
    plt.grid(True)
    
    plt.show()

if __name__ == '__main__':
    file_path = '/home/ale/ros2_ws/src/SIMPLE_SYSTEM_CONTROL/system_control/data_record.csv'  # Update this path if necessary
    plot_joint_states(file_path)
