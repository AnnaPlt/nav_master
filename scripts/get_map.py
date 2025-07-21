#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid

def costmap_callback(msg):
    # Estrai dimensioni della griglia
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution

    # Converti i dati (da lista a matrice 2D)
    data = np.array(msg.data).reshape((height, width))

    # Rimpiazziamo valori sconosciuti (-1) con NaN
    data = np.where(data == -1, np.nan, data)

    # Plot heatmap
    plt.figure(figsize=(8, 6))
    plt.imshow(data, origin='lower', cmap='viridis')
    plt.title("Costmap Heatmap")
    plt.xlabel("X (celle)")
    plt.ylabel("Y (celle)")
    cbar = plt.colorbar(label='Costo (0-100)')
    plt.grid(False)
    plt.savefig("costmap_heatmap.png")

def listener():
    rospy.init_node('costmap_visualizer', anonymous=True)
    rospy.Subscriber("/local_costmap/costmap", OccupancyGrid, costmap_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

