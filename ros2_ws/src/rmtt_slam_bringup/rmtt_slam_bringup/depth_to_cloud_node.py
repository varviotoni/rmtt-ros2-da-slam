#!/usr/bin/env python3
"""Monocular Metric Depth + Forward ToF Scale Calibration to 3D PointCloud2 node."""

import math
import os
import threading
from typing import List, Optional

import cv2
import numpy as np
import rclpy
import torch
import torch.nn.functional as F
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from transformers import AutoImageProcessor, AutoModelForDepthEstimation


class DepthToCloudNode(Node):
    """Runs Depth Anything V2 (Metric Indoor) calibrated by /tof_ext to publish /cloud_obstacles."""

    MODEL_ID = 'depth-anything/Depth-Anything-V2-Metric-Indoor-Base-hf'

    # Default RMTT camera intrinsics for 960 x 720
    REF_WIDTH = 960.0
    REF_HEIGHT = 720.0
    FX = 921.17
    FY = 919.02
    CX = 460.0
    CY = 351.0
    DIST_COEFFS = [-0.034, 0.052, -0.0015, 0.0008, 0.0]

    def __init__(self) -> None:
        super().__init__('depth_to_cloud_node')

        # Configurable HSV threshold parameters for high-contrast towers
        self.declare_parameter('hsv_lower', [0, 0, 0])
        self.declare_parameter('hsv_upper', [180, 255, 255])
        self.declare_parameter('enable_undistortion', True)
        self.declare_parameter('dist_coeffs', self.DIST_COEFFS)
        self.enable_undistortion: bool = bool(
            self.get_parameter('enable_undistortion').value
        )

        self._cached_shape = None
        self._map1 = None
        self._map2 = None
        self._new_camera_matrix = None

        self.bridge = CvBridge()
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.dtype = torch.float16 if self.device.type == 'cuda' else torch.float32

        # Publisher and Subscribers created immediately so topics are advertised on startup
        self.cloud_pub = self.create_publisher(PointCloud2, '/cloud_obstacles', 1)
        self.depth_image_pub = self.create_publisher(Image, '/depth/image_raw', 1)
        self.depth_vis_pub = self.create_publisher(Image, '/depth/image_vis', 1)
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self._image_callback, 1
        )

        self.processor: Optional[AutoImageProcessor] = None
        self.model: Optional[AutoModelForDepthEstimation] = None
        self._model_ready = False
        self._model_thread = threading.Thread(target=self._load_model, daemon=True)
        self._model_thread.start()

    def _load_model(self) -> None:
        """Load Depth Anything V2 Metric Indoor onto cuda:0 in torch.float16."""
        try:
            self.get_logger().info(
                f'Loading {self.MODEL_ID} onto {self.device} ({self.dtype})...'
            )
            processor = AutoImageProcessor.from_pretrained(self.MODEL_ID)
            model = AutoModelForDepthEstimation.from_pretrained(
                self.MODEL_ID, torch_dtype=self.dtype
            ).to(self.device)
            model.eval()
            self.processor = processor
            self.model = model
            self._model_ready = True
            self.get_logger().info(f'Successfully loaded {self.MODEL_ID}.')
        except Exception as exc:
            self.get_logger().error(f'Failed to load Depth Anything V2 model: {exc}')

    def _undistort_image(self, img: np.ndarray):
        """Undistort image using cached cv2 remap for sub-millisecond real-time execution."""
        height, width = img.shape[:2]
        if self._cached_shape != (height, width):
            scale_x = float(width) / self.REF_WIDTH
            scale_y = float(height) / self.REF_HEIGHT
            k = np.array(
                [
                    [self.FX * scale_x, 0.0, self.CX * scale_x],
                    [0.0, self.FY * scale_y, self.CY * scale_y],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            d = np.array(self.get_parameter('dist_coeffs').value, dtype=np.float64)

            new_k, _ = cv2.getOptimalNewCameraMatrix(
                k, d, (width, height), 0.0, (width, height)
            )
            self._map1, self._map2 = cv2.initUndistortRectifyMap(
                k, d, None, new_k, (width, height), cv2.CV_16SC2
            )
            self._new_camera_matrix = new_k
            self._cached_shape = (height, width)

        undistorted = cv2.remap(
            img, self._map1, self._map2, interpolation=cv2.INTER_LINEAR
        )
        return undistorted, self._new_camera_matrix

    def _image_callback(self, msg: Image) -> None:
        """Run metric depth estimation on undistorted frame, segment towers, and back-project to 3D."""
        if not self._model_ready or self.model is None or self.processor is None:
            return

        bgr_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = bgr_img.shape[:2]

        # Apply lens undistortion if enabled
        if self.enable_undistortion:
            bgr_img, cam_matrix = self._undistort_image(bgr_img)
            fx = float(cam_matrix[0, 0])
            fy = float(cam_matrix[1, 1])
            cx = float(cam_matrix[0, 2])
            cy = float(cam_matrix[1, 2])
        else:
            scale_x = float(width) / self.REF_WIDTH
            scale_y = float(height) / self.REF_HEIGHT
            fx = self.FX * scale_x
            fy = self.FY * scale_y
            cx = self.CX * scale_x
            cy = self.CY * scale_y

        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        # Predict dense metric depth map
        inputs = self.processor(images=rgb_img, return_tensors='pt')
        pixel_values = inputs['pixel_values'].to(device=self.device, dtype=self.dtype)

        with torch.no_grad():
            outputs = self.model(pixel_values=pixel_values)
            predicted_depth = outputs.predicted_depth

            # Interpolate to original image resolution (H, W)
            depth_resized = F.interpolate(
                predicted_depth.unsqueeze(1).float(),
                size=(height, width),
                mode='bicubic',
                align_corners=False,
            ).squeeze(1).squeeze(0)

        depth_metric = depth_resized.detach().cpu().numpy().astype(np.float32)

        # Publish 32FC1 metric depth image (meters)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_metric, encoding='32FC1')
        depth_msg.header = msg.header
        self.depth_image_pub.publish(depth_msg)

        # Publish colorized depth map for direct visual inspection in RViz2 / rqt_image_view
        depth_clipped = np.clip(depth_metric, 0.15, 4.5)
        depth_norm = ((depth_clipped - 0.15) / (4.5 - 0.15) * 255.0).astype(np.uint8)
        depth_vis_bgr = cv2.applyColorMap(255 - depth_norm, cv2.COLORMAP_INFERNO)
        vis_msg = self.bridge.cv2_to_imgmsg(depth_vis_bgr, encoding='bgr8')
        vis_msg.header = msg.header
        self.depth_vis_pub.publish(vis_msg)

        # HSV thresholding for high-contrast towers
        hsv_lower_param: List[int] = list(self.get_parameter('hsv_lower').value)
        hsv_upper_param: List[int] = list(self.get_parameter('hsv_upper').value)
        hsv_lower = np.array(hsv_lower_param, dtype=np.uint8)
        hsv_upper = np.array(hsv_upper_param, dtype=np.uint8)

        # Extract only rows within central horizontal band v in [H/2 - 60, H/2 + 60]
        v_min = max(0, (height // 2) - 60)
        v_max = min(height, (height // 2) + 60)

        # Downsample pixel grid by stride 4 for speed
        v_indices = np.arange(v_min, v_max, 4, dtype=np.int32)
        u_indices = np.arange(0, width, 4, dtype=np.int32)

        band_bgr = bgr_img[np.ix_(v_indices, u_indices)]
        band_hsv = cv2.cvtColor(band_bgr, cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(band_hsv, hsv_lower, hsv_upper) > 0

        band_depth = depth_metric[np.ix_(v_indices, u_indices)]
        valid_mask = (
            hsv_mask
            & np.isfinite(band_depth)
            & (band_depth >= 0.15)
            & (band_depth <= 4.5)
        )

        uu, vv = np.meshgrid(u_indices.astype(np.float32), v_indices.astype(np.float32))
        u_valid = uu[valid_mask]
        v_valid = vv[valid_mask]
        z_cam = band_depth[valid_mask].astype(np.float32)

        # 3D Back-Projection to base_stabilized (ROS FLU)
        x_flu = z_cam
        y_flu = -((u_valid - cx) * z_cam) / fx
        z_flu = -((v_valid - cy) * z_cam) / fy

        points = np.stack((x_flu, y_flu, z_flu), axis=-1).astype(np.float32)

        cloud_msg = self._create_pointcloud2(points, msg.header.stamp, 'base_stabilized')
        self.cloud_pub.publish(cloud_msg)

    @staticmethod
    def _create_pointcloud2(
        points: np.ndarray, stamp, frame_id: str
    ) -> PointCloud2:
        """Pack (N, 3) float32 XYZ array into a sensor_msgs/msg/PointCloud2 message."""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.height = 1
        msg.width = int(points.shape[0])
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
        return msg


def main(args=None) -> None:
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=args)
    node = DepthToCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
