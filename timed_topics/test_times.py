import time

import rclpy
from rclpy.node import Node
from timed_topics.msg import TimedLowState

import numpy as np
import matplotlib.pyplot as plt

class TimeRepublisher(Node):
    def __init__(self):
        super().__init__('time_republisher')
        self.create_subscription(
            TimedLowState,
            '/timedlowstate',
            self.cb_timedlow,
            10)
        
        self.times = []

    def cb_timedlow(self, msg: TimedLowState):
        t = msg.stamp
        secs = t.sec + t.nanosec / 1e9
        self.times.append(secs)
        
        if len(self.times) == 10000:
            self.get_logger().info(f"Got enough samples, evaluating...")
            # Descending...
            self.times.sort(reverse=True)
            times = np.array(self.times)
            dt = (times[:-1] - times[1:]) * 1000 # s -> ms
            
            plt.figure()
            plt.hist(dt, bins=100, color='blue', alpha=0.7)
            plt.xlabel('Delta t [ms]')
            plt.ylabel('Frequency')
            plt.title(f'Histogram dt, mean: {np.mean(dt)} std: {np.std(dt)}')
            plt.grid(True)
            plt.savefig('/tmp/dt_hist.png')
            plt.show()

            time.sleep(100000)
        

def main(args=None):
    rclpy.init(args=args)
    node = TimeRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()