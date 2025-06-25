"""This code waits until an obstacle in front is lifted and then drives straight, until it sees another obstacle close by.

"""

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile, QoSReliabilityPolicy



class StartSignalHandler(Node):
    def __init__(self):
        super().__init__('start_signal_handler')
        
        
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan, 
            'scan', 
            self.scan_callback, 
            qos_profile
        )

        # self.subscription = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.scan_callback,
        #     10
        # )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.last_raw_scan_data = LaserScan()
        self.waiting_for_start = True
        
    def scan_callback(self, scan_data):
        self.last_raw_scan_data = scan_data
        
    def drive_straight(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.
        self.publisher.publish(msg)

    def turn_left(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.3
        self.publisher.publish(msg)
    
    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.3
        self.publisher.publish(msg)
    
    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        
    def sort_data(self, raw_data):
        
        msg = raw_data
        if len(msg.ranges) != 360:
            print(f'The number of measurements is {len(msg.ranges)}, not 360, which is the expected value.'
                  + '\nThis code is hardcoded to work with only 360 measurements')
            return 0
        
        # Replace 0.0 with math.inf
        cleaned_ranges = [r if r != 0.0 else math.inf for r in msg.ranges]
            
        return {
            'front':  min(min(cleaned_ranges[0:20] + cleaned_ranges[340:360]), 10),
            'fleft': min(min(cleaned_ranges[20:40]), 10),
            'left':  min(min(cleaned_ranges[41:80]), 10),
            'fright':  min(min(cleaned_ranges[321:339]), 10),
            'right':   min(min(cleaned_ranges[310:320]), 10),
            'bright':   min(min(cleaned_ranges[240:309]), 10),
        }

        
    def control_loop(self):
        data_regions = self.sort_data(self.last_raw_scan_data)
        if data_regions == 0:
            print("asdfasdf")
            return
        
        d = 0.2
        if self.waiting_for_start:
            if data_regions['front'] < d or data_regions['fleft'] < d or data_regions['fright'] < d:
                print("Waiting for start") # debugging, may be removed
            else:
                self.waiting_for_start = False
                print("Ready for start!")
        else:
            if data_regions['front'] < d or data_regions['fleft'] < d or data_regions['fright'] < d:
                self.stop_robot()
                print("Obstacle, stopping")
                #TODO End the script gracefully
                rclpy.shutdown()  # Shutdown ROS2 cleanly
            else:
                self.drive_straight()
                print("Driving till obstacle")


    

    


def main(args=None):
    rclpy.init(args=args)
    node = StartSignalHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Stopping robot...')
    finally:
        node.stop_robot()  # 🛑 Stop before exiting
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
