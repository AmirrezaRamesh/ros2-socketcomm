#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from collatz_core.msg import Num
import matplotlib.pyplot as plt

class CollatzVisualizer(Node):
    def __init__(self):
        super().__init__('collatz_visualizer')
        
        #store values
        self.values = []

        #subscribe to the data
        self.create_subscription(Num, 'collatz_number', self.update_plot, 10)

        #configure the plot
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-')
        self.ax.set_xlabel('step')
        self.ax.set_ylabel('/collatz_number')
        self.ax.set_title('Progress')

    def update_plot(self, data):
        #append new data to values
        self.values.append(data.num)

        #assign new data
        self.line.set_xdata(range(len(self.values)))
        self.line.set_ydata(self.values)

        #update limits of the plot based of new data
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.set_ylim(0, max(self.values) * 1.2)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):

    rclpy.init(args=args)
    node = CollatzVisualizer()
    rclpy.spin(node)

    plt.ioff()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


