"""
ROS 2 service node to detect a specific traffic sign using template matching
and an image pyramid for scale invariance.

Resources:
- Service structure: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html
- Image Pyramid: https://pyimagesearch.com/2015/03/16/image-pyramids-with-python-and-opencv
- Template Matching: https://docs.opencv.org/4.x/d4/dc6/tutorial_py_template_matching.html
"""

from autorace_real_interfaces.srv import SignMsg
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np


class DetectTunnelSign(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            SignMsg,
            'detect_tunnel_sign',
            self.match_template
        )
        self.br = CvBridge()

        self.build_target = 1  # 1 is real bot, 0 is simulated bot
        if self.build_target == 1:
            template_path = "/root/ros2_ws/src/rm-202356/stage_5/autorace_real/resource/sign_real_rect-32-35.png"
        else:
            template_path = "/root/ros2_ws/src/rm-202356/stage_5/autorace_real/resource/sign_sim_rect-32-35.png"

        self.template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
        self.tw, self.th = self.template.shape[::-1]
        self.scale = 1.2  # the size differences of the pyramid, has to be greater than 1
        # smallest size should be size of template
        self.min_size = (self.tw, self.th)
        self.method = cv2.TM_CCOEFF_NORMED
        self.detection_threshold = 0.75

    def pyramid(self, image):
        """
        Generator to yield images in an image pyramid, scaled down by the factor self.scale
        until the minimum size is reached.

        Args:
            image (numpy.ndarray): Input grayscale image.

        Yields:
            numpy.ndarray: Resized image at each pyramid level.
        """
        yield image
        while True:
            w = int(image.shape[1] / self.scale)
            h = int(image.shape[0] * (w / image.shape[1]))
            image = cv2.resize(image, (w, h))

            if image.shape[0] < self.min_size[1] or image.shape[1] < self.min_size[0]:
                break
            yield image

    def match_template(self, request, response):
        """
        Performs template matching over an image pyramid and returns True if the template
        was detected.

        Args:
            gray (numpy.ndarray): Grayscale image to search within.

        Returns:
            bool: True if template detected, False otherwise.
            tuple: Coordinates of the bounding box if detected, else None.
        """

        if request.image is None:
            print("Shouldn't get empty image, but got it, returning with false")
            response.detected = False
            return response

        frame = self.br.imgmsg_to_cv2(request.image, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        # Improve contrast under different lighting
        gray = cv2.equalizeHist(gray)

        best_val = None
        best_loc = None
        best_scale_factor = None

        # Loop over image pyramid layers
        for layer in self.pyramid(gray):
            # Perform template matching
            res = cv2.matchTemplate(layer, self.template, self.method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

            # Depending on method, select min or max correlation value and location
            if self.method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                val = min_val
                loc = min_loc
                is_better = best_val is None or val < best_val
            else:
                val = max_val
                loc = max_loc
                is_better = best_val is None or val > best_val

            if is_better:
                best_val = val
                best_loc = loc
                # Scale factor to convert coordinates back to original image scale
                scale_factor = gray.shape[1] / float(layer.shape[1])
                best_scale_factor = scale_factor

        # Decide if detection is confident enough
        detected = False
        bbox = None
        if best_val is not None:
            # For correlation-based methods higher is better, for difference lower is better
            if (self.method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED] and best_val <= (1 - self.detection_threshold)) or \
               (self.method not in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED] and best_val >= self.detection_threshold):
                detected = True

                # for logging and drawing a boundary box
                top_left = (int(best_loc[0] * best_scale_factor),
                            int(best_loc[1] * best_scale_factor))
                bottom_right = (int((best_loc[0] + self.tw) * best_scale_factor),
                                int((best_loc[1] + self.th) * best_scale_factor))
                bbox = (top_left, bottom_right)

        if detected and bbox is not None:
            cv2.rectangle(frame, bbox[0], bbox[1], (0, 255, 0), 2)
            self.get_logger().info('Pattern detected')
        else:
            self.get_logger().info('Pattern not detected')

        cv2.imshow("Pattern Match Output", frame)
        cv2.waitKey(1)

        response.detected = detected
        return response


def main(args=None):
    rclpy.init(args=args)
    minimal_service = DetectTunnelSign()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
