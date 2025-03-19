#!/usr/bin/python3
import rospy
from gesture_recognizer.msg import Metric
from collections import deque
import pyqtgraph as pg
from PyQt5 import QtCore
import numpy as np

class MetricPlotter:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('metric_plotter', anonymous=True)
        
        # Data storage
        self.max_points = 300
        self.timestamps = deque(maxlen=self.max_points)
        self.processing_delays = deque(maxlen=self.max_points)
        self.network_latencies = deque(maxlen=self.max_points)
        self.start_time = rospy.get_time()
        
        # Set up PyQtGraph window
        self.app = pg.mkQApp()  # Create Qt application
        self.win = pg.GraphicsLayoutWidget(title="Real-Time Metrics")
        self.win.resize(800, 600)
        self.win.show()
        
        # Create plots
        self.plot1 = self.win.addPlot(title="Processing Delay", row=0, col=0)
        self.plot1.setLabel('left', 'Seconds')
        self.plot1.setLabel('bottom', 'Time (s)')
        self.plot1.setRange(xRange=[0, 60], yRange=[0, 1])
        self.curve1 = self.plot1.plot(pen='b')
        
        self.plot2 = self.win.addPlot(title="Network Latency", row=1, col=0)
        self.plot2.setLabel('left', 'Seconds')
        self.plot2.setLabel('bottom', 'Time (s)')
        self.plot2.setRange(xRange=[0, 60], yRange=[0, 1])
        self.curve2 = self.plot2.plot(pen='r')
        
        # Subscribe to metrics
        rospy.Subscriber('/metrics', Metric, self.metrics_callback)
        
        # Timer for plot updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # Update every 100ms
    
    def metrics_callback(self, msg):
        current_time = rospy.get_time() - self.start_time + 0.01
        self.timestamps.append(current_time)
        self.processing_delays.append(msg.processing_delay)
        self.network_latencies.append(msg.network_latency)
    
    def update_plot(self):
        self.curve1.setData(list(self.timestamps), list(self.processing_delays))
        self.curve2.setData(list(self.timestamps), list(self.network_latencies))
    
    def run(self):
        if not rospy.is_shutdown():
            self.app.exec_()  # Start Qt event loop

if __name__ == '__main__':
    try:
        plotter = MetricPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass