"""Only there to test the tunnel_detector_srv

https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html"""

from autorace_real_interfaces.srv import SignMsg
from sensor_msgs.msg import Image
import sys
import rclpy
from rclpy.node import Node



class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SignMsg, 'detect_tunnel_sign')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
        self.req = SignMsg.Request()
        
        self.img_data_listener = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            #'/camera/image_raw',
            self.store_image,
            10
        )
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.future = None
        self.current_frame = None
        
    def store_image(self, data):
        self.current_frame = data
        
    
    def timer_callback(self):
        if self.current_frame is not None:
            self.send_request()

    def send_request(self):
        if self.current_frame is None:
            self.get_logger().info('Waiting for camera image data...')
            return # this lead to an error
        
        self.req.image = self.current_frame
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'There was a sign detected: {response.detected}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')



def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    rclpy.spin(minimal_client)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()