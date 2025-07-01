"""This code follows the lane marking on either side of the lane"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as ocv
import numpy as np
from collections import namedtuple
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from autorace_real_interfaces.srv import SignMsg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class ImageSubscriber(Node):

    current_frame_raw = None

    def __init__(self):
        super().__init__('EnterTunnelReal')

        # create a service client to detect the tunnel sign
        self.cli = self.create_client(SignMsg, 'detect_tunnel_sign')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = SignMsg.Request()
        self.future = None

        # declare parameters
        self.declare_parameter('line', 'white')
        self.line_to_follow = self.get_parameter(
            'line').get_parameter_value().string_value

        if self.line_to_follow not in ['white', 'yellow']:
            raise ValueError(
                f'Invalid line color: {self.line_to_follow}. Choose either "white" or "yellow".')

        # init instance variables
        self.get_logger().info(f'Following line: {self.line_to_follow}')
        match self.line_to_follow:
            case 'white':
                self.driving_mode = 1  # 0 stopping, 1 following lines right, 2 following lines left
            case 'yellow':
                self.driving_mode = 2  # 0 stopping, 1 following lines right, 2 following lines left

        # init the last driving mode to the desired side of the lane
        self.last_driving_mode = self.driving_mode
        self.driving_mode = 3  # Wait for the sign before starting to drive

        self.laser_scan = None
        self.lines_right = []
        self.lines_left = []
        self.build_target = 1  # 1 is real bot, 0 is simulated bot
        # setup main loop
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.main_loop)
        # subscribe to image data
        if (self.build_target == 1):
            self.img_data_listener = self.create_subscription(
                Image,
                '/camera/image_rect_color',
                self.store_image,
                10)
        else:
            self.img_data_listener = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.store_image,
                10)
        self.br = CvBridge()
        # create a listener for laser data
        qos_profile = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.scan_listener = self.create_subscription(
            LaserScan,
            '/scan',
            self.store_scan,
            qos_profile)
        # create a publisher for cmd_vel
        self.drive_control = self.create_publisher(Twist, '/cmd_vel', 7)

    def store_image(self, data):
        self.current_frame_raw = data

    def store_scan(self, msg):
        self.laser_scan = np.asarray(msg.ranges)

    def main_loop(self):
        if self.laser_scan is None:
            self.get_logger().info('Waiting for laser scan data...')
            return
        if self.current_frame_raw is None:
            self.get_logger().info('Waiting for camera image data...')
            return
        else:
            self.process_image()
            if self.is_emergency_brake_active() and self.driving_mode != 0:
                # transitioning to emergency stop
                self.last_driving_mode = self.driving_mode
                self.driving_mode = 0
            elif not self.is_emergency_brake_active() and self.driving_mode == 0:
                # transitioning from emergency stop to last driving mode
                self.driving_mode = self.last_driving_mode
            elif self.driving_mode != 0:
                # driving normally and storing last driving mode
                self.last_driving_mode = self.driving_mode
            else:
                # if we are in emergency stop, we do not want to change the driving mode
                pass

            match self.driving_mode:
                case 0:  # stopped
                    command = Twist()
                    command.linear.x = 0.00
                    self.drive_control.publish(command)
                case 1:  # following lines right
                    self.driving_mode = self.follow_lines_right()
                case 2:  # following lines left
                    self.driving_mode = self.follow_lines_left()
                case 3:  # waiting for the sign to be detected
                    # TODO: need to implement fault handling on the request and make sure that a request is not sent while one is pending
                    self.send_request()

            self.get_logger().info(
                f'Driving mode: {self.driving_mode}, Last driving mode: {self.last_driving_mode}')

    def is_emergency_brake_active(self) -> bool:
        '''Check if the emergency brake assist (EBA) is active based on laser scan data.'''
        if self.laser_scan[0] > 0.001 and self.laser_scan[0] < 0.2:
            self.get_logger().info(f'Emergency stop, too close to an obstacle!')
            command = Twist()
            command.linear.x = 0.00
            self.drive_control.publish(command)
            return True  # stop roboter
        else:
            return False  # continue driving

    def process_image(self):
        current_frame_bgr = self.br.imgmsg_to_cv2(self.current_frame_raw)
       # Convert BGR to HSV
        current_frame_hsv = ocv.cvtColor(
            current_frame_bgr, ocv.COLOR_RGB2HSV)

        # define range of blue color in HSV
        hsv_white_low = np.array([0, 0, 190])
        hsv_white_high = np.array([180, 80, 255])

        # Threshold the HSV image to get only blue colors
        white_mask = ocv.inRange(
            current_frame_hsv, hsv_white_low, hsv_white_high)

        # Bitwise-AND mask and original image
        masked_image = ocv.bitwise_and(
            current_frame_hsv, current_frame_hsv, mask=white_mask)
        masked_image = ocv.cvtColor(masked_image, ocv.COLOR_HSV2BGR)
        masked_image = ocv.resize(masked_image, (240, 160))

        display_image = ocv.cvtColor(current_frame_bgr, ocv.COLOR_RGB2BGR)
        display_image = ocv.resize(display_image, (240, 160))
        ocv.imshow("masked image", masked_image)
        ocv.waitKey(1)
        eval_image = ocv.cvtColor(display_image, ocv.COLOR_BGR2GRAY)
        eval_image = ocv.Canny(eval_image, 130, 200)
        display_image = eval_image
        display_image = ocv.cvtColor(display_image, ocv.COLOR_GRAY2BGR)
        ocv.imshow("edges", display_image)
        ocv.waitKey(1)

        self.lines_right = []
        # parse the bottom few lines of the gradient image. FOV is right half and bottom 5 lines
        for image_line in eval_image[-5:, -120:]:
            line_edge_idx = np.argmax(image_line)
            if line_edge_idx > 0:
                self.lines_right.append(line_edge_idx)

        self.lines_left = []
        # parse the bottom few lines of the gradient image. FOV is right half and bottom 5 lines
        for image_line in eval_image[-5:, 120::-1]:
            line_edge_idx = np.argmax(image_line)
            if line_edge_idx > 0:
                self.lines_left.append(120 - line_edge_idx)

    def follow_lines_right(self) -> int:
        latral_offset = 0
        if len(self.lines_right) > 0:
            self.lines_right = np.array(self.lines_right)
            self.get_logger().info(f'Lines right: {self.lines_right}')
            # offset from the center of the image, since were only looking at the right half of the image
            if self.build_target == 1:
                learned_offset_bias = 205 - 120  # real robot
            else:
                learned_offset_bias = 180 - 120  # simulated robot
            latral_offset = self.lines_right.mean() - learned_offset_bias
            self.steer_robot(latral_offset)
            return 1  # keep driving
        else:
            self.get_logger().info(f'No lines right detected, switching to left')
            return 2  # follow the left line

    def follow_lines_left(self) -> int:
        latral_offset = 0
        if len(self.lines_left) > 0:
            self.lines_left = np.array(self.lines_left)
            self.get_logger().info(f'Lines left: {self.lines_left}')
            # offset from the center of the image, since were only looking at the left half of the image
            if self.build_target == 1:
                learned_offset_bias = 45  # real robot
            else:
                learned_offset_bias = 60  # simulated robot
            latral_offset = self.lines_left.mean() - learned_offset_bias
            self.steer_robot(latral_offset)
            return 2  # keep driving
        else:
            self.get_logger().info(f'No lines left detected, switching to right')
            return 1  # follow the right line

    def steer_robot(self, latral_offset: float):
        self.get_logger().info(f'Latral offset: {latral_offset / 20}')
        command = Twist()
        if self.build_target == 1:
            command.angular.z = -0.35 * \
                np.tanh(latral_offset / 15)  # real robot
        else:
            command.angular.z = -0.35 * \
                np.tanh(latral_offset / 20)  # simulated robot
        command.linear.x = 0.15 * np.exp(-np.abs(latral_offset) / 30)
        self.drive_control.publish(command)

    def send_request(self):
        if (self.future is None or self.future.done()):
            self.get_logger().info('Sending request to detect tunnel sign...')
            self.req.image = self.current_frame_raw
            self.future = self.cli.call_async(self.req)
            self.future.add_done_callback(self.response_callback)
        else:
            self.get_logger().info('Waiting for response from service...')

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f'There was a sign detected: {response.detected}')
            if response.detected:
                self.driving_mode = 1
                self.last_driving_mode = 1
            else:
                pass  # keep looking for THE sign
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
