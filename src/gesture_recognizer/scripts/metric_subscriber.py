#!/usr/bin/python3
import rospy
from gesture_recognizer.msg import Metric
import matplotlib.pyplot as plt
from collections import deque

class MetricPlotter:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('metric_plotter', anonymous=True)
        
        # Data storage (using deque for a sliding window)
        self.max_points = 100  # Number of points to display
        self.processing_delays = deque(maxlen=self.max_points)
        self.network_latencies = deque(maxlen=self.max_points)
        self.timestamps = deque(maxlen=self.max_points)
        self.start_time = rospy.get_time()
        
        # Set up matplotlib plot
        plt.ion()  # Interactive mode for real-time updates
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 6))
        self.line1, = self.ax1.plot([], [], 'b-', label='Processing Delay')
        self.line2, = self.ax2.plot([], [], 'r-', label='Network Latency')
        
        # Configure plots
        self.ax1.set_title('Processing Delay')
        self.ax1.set_ylabel('Seconds')
        self.ax1.legend()
        self.ax2.set_title('Network Latency')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Seconds')
        self.ax2.legend()
        
        # Subscribe to the /metrics topic
        rospy.Subscriber('/metrics', Metric, self.metrics_callback)
    
    def metrics_callback(self, msg):
        # Extract data from the message
        processing_delay = msg.processing_delay
        network_latency = msg.network_latency
        
        # Append data to storage
        current_time = rospy.get_time() - self.start_time
        self.timestamps.append(current_time)
        self.processing_delays.append(processing_delay)
        self.network_latencies.append(network_latency)
        
        # Update plot data
        self.line1.set_data(self.timestamps, self.processing_delays)
        self.line2.set_data(self.timestamps, self.network_latencies)
        
        # Adjust plot limits
        if self.timestamps:
            self.ax1.set_xlim(min(self.timestamps), max(self.timestamps))
            self.ax2.set_xlim(min(self.timestamps), max(self.timestamps))
            if self.processing_delays:
                self.ax1.set_ylim(min(self.processing_delays) * 0.9, max(self.processing_delays) * 1.1)
            if self.network_latencies:
                self.ax2.set_ylim(min(self.network_latencies) * 0.9, max(self.network_latencies) * 1.1)
        
        # Redraw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        plotter = MetricPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        plt.close()

