"""This code follows a wall, and is robust against obstacles.

The inspiration for this code is from here, however, there are significant changes in the main logic.
https://www.theconstruct.ai/wall-follower-algorithm/

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
        
        self.allowed_states = ["turn_right", "turn_left", "drive_straight"]

        self.last_state = "no_state"
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
        msg.linear.x = 0.03
        msg.angular.z = 0.3
        self.publisher.publish(msg)
    
    def turn_right(self):
        msg = Twist()
        msg.linear.x = 0.03
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
        
    def calc_state(self, data_regions):
        def change_state(state):

            if state is not self.state:
                print(f'Changing state from {self.state} to {state}')
                self.state = state
                
                
        
        
        d = 0.5
        
        if min(data_regions.values()) > d:
            return 0
        elif data_regions['front'] < d or data_regions['fleft'] < d or data_regions['fright'] < d:
            return 1
        elif data_regions['right'] < d + 0.1:
            print('follow wall')
            return 0
        elif data_regions['bright'] < d + 0.05:
            return 2
        else:
            return 2
        
    
    def change_state(self, state):
        if state not in self.allowed_states:
            match state:
                case 0:
                    state = 'drive_straight'
                case 1:
                    state = 'turn_left'
                case 2:
                    state = 'turn_right'
                case _:
                    print('Unknown state')
                    return
        if state is not self.last_state:
            print(f'Changing state from {self.last_state} to {state}')
            self.last_state = state
            
        return state

        
    def control_loop(self):
        data_regions = self.sort_data(self.last_raw_scan_data)
        if data_regions == 0:
            print("asdfasdf")
            return
        
        d = 0.2
        
        if self.waiting_for_start:
            if data_regions['front'] < d or data_regions['fleft'] < d or data_regions['fright'] < d:    
                print("Waiting for start") # debugging, may be removed
                return
            else:
                self.waiting_for_start = False
                print("Ready for start!")
        
        state = self.calc_state(data_regions)
        state = self.change_state(state)
        match state:
            case 'turn_right':
                self.turn_right()
            case 'turn_left':
                self.turn_left()
            case 'drive_straight':
                self.drive_straight()
            case _:
                print('No state')


    

    


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
    print("Starting to follow wall...")
    main()
