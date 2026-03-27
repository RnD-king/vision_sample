from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
# 메세지
from message_filters import Subscriber, ApproximateTimeSynchronizer

import rclpy as rp
import numpy as np
import cv2

class DepthTestNode(Node):
    def __init__(self):
        super().__init__('depth_test')

        self.declare_parameter("depth_min", 30.0)
        self.declare_parameter("depth_max", 500.0)

        self.depth_min = float(self.get_parameter("depth_min").value)
        self.depth_max = float(self.get_parameter("depth_max").value)

        # 컬러, 깊이 영상 동기화
        self.bridge = CvBridge()
        color_sub = Subscriber(self, Image, '/camera/color/image_raw') # 칼라
        depth_sub = Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw') # 깊이
        self.sync = ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=5, slop=0.1)
        self.sync.registerCallback(self.image_callback)
        
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params):
        next_depth_min = self.depth_min
        next_depth_max = self.depth_max

        for p in params:
            if p.name == "depth_min":
                next_depth_min = float(p.value)
            elif p.name == "depth_max":
                next_depth_max = float(p.value)

        if next_depth_min >= next_depth_max:
            return SetParametersResult(
                successful=False,
                reason="depth_min must be less than depth_max",
            )

        self.depth_min = next_depth_min
        self.depth_max = next_depth_max
        return SetParametersResult(successful=True)

    def image_callback(self, color_msg: Image, depth_msg: Image):

        # 영상 받아오기
        frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough').astype(np.float32)

        valid_depth = np.isfinite(depth) & (depth > self.depth_min) & (depth < self.depth_max)
        frame[~valid_depth] = 0

        cv2.imshow('Raw Hurdle Mask', frame) # 기준 거리 이내
        cv2.waitKey(1)

def main():
    rp.init()
    node = DepthTestNode()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
