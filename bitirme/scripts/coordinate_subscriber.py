#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from bitirme.msg import Coordinate

# create list to keep coordinates and time stamps
x_coords = []
y_coords = []
z_coords = []
real_time_stamps = []

# create variables to keep start time
start_time = None

def coordinate_callback(msg):
    global x_coords, y_coords, z_coords, real_time_stamps, start_time
    
    # add coordinates to lists
    x_coords.append(msg.x)
    y_coords.append(msg.y)
    z_coords.append(msg.z)
    
    # save the real time stamps
    if start_time is None:
        start_time = rospy.Time.now()
    
    current_time = rospy.Time.now()
    real_time_stamps.append((current_time - start_time).to_sec())

def plot_coordinates():
    rospy.init_node('plot_coordinates', anonymous=True)
    rospy.Subscriber('coordinates', Coordinate, coordinate_callback)

    plt.figure(figsize=(10, 8))
    start_time = rospy.Time.now()  # make start time as a constant time

    while not rospy.is_shutdown():
        plt.clf()  # clean the graphs
        
        # graph updates
        plt.subplot(3, 1, 1)
        plt.plot(real_time_stamps, x_coords, label='X Coordinate (Msg)')
        plt.plot(real_time_stamps, [0.5]*len(real_time_stamps), label='X Coordinate (Desired)', linestyle='dashed')
        plt.xlabel('Time (seconds)')
        plt.ylabel('X Coordinate Value')
        plt.title('X Coordinate')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(3, 1, 2)
        plt.plot(real_time_stamps, y_coords, label='Y Coordinate (Msg)')
        plt.plot(real_time_stamps, [0.5]*len(real_time_stamps), label='Y Coordinate (Desired)', linestyle='dashed')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Y Coordinate Value')
        plt.title('Y Coordinate')
        plt.legend()
        plt.grid(True)

        plt.subplot(3, 1, 3)
        plt.plot(real_time_stamps, z_coords, label='Z Coordinate (Msg)')
        plt.plot(real_time_stamps, [0.5]*len(real_time_stamps), label='Z Coordinate (Desired)', linestyle='dashed')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Z Coordinate Value')
        plt.title('Z Coordinate')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.pause(0.1)  # wait to update the graph 
        
        # clean old values
        while real_time_stamps and real_time_stamps[0] < real_time_stamps[-1] - 10:
            x_coords.pop(0)
            y_coords.pop(0)
            z_coords.pop(0)
            real_time_stamps.pop(0)

if __name__ == '__main__':
    try:
        plot_coordinates()
    except rospy.ROSInterruptException:
        pass

