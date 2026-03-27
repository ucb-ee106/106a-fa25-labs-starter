import rclpy
import cv2
import os
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.bridge = CvBridge()

        # Load YOLO model
        package_share_dir = get_package_share_directory('perception')
        model_path = os.path.join(package_share_dir, 'utilities', 'segment_bounding_box_cones.pt')
        self.model = YOLO(model_path)

        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 1)
        self.cone_position_pub = self.create_publisher(PointStamped, '/goal_point', 1)
        self.camera_intrinsics = None

        if len(sys.argv) > 1 and sys.argv[1] == 'display_yolo':
            self.display_yolo = True
            plt.ion()
            self.fig, self.ax = plt.subplots(1, 1, figsize=(12, 8))
            self.fig.canvas.manager.set_window_title('YOLO Cone Segmentation')
        else:
            self.display_yolo = False

        self.get_logger().info('Image Subscriber Node initialized')
        self.camera_intrinsics = 'bonk'

    def image_callback(self, msg):
        if self.camera_intrinsics is None:
            self.get_logger().warn("No Camera Intrinsics!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        results = self.model(cv_image, verbose=False)

        if self.display_yolo:
            self.visualize_results(cv_image, results)

        for result in results:
            if result.masks is not None:
                masks = result.masks.data.cpu().numpy()

                img_height, img_width = cv_image.shape[:2]
                for i, mask in enumerate(masks):

                    # TODO: Get number of pixels in mask
                    pixel_count = 0. 

                    CONE_AREA = 0.0208227849

                    # TODO: Get depth of image 
                    depth = 0.

                    self.get_logger().info(f'Cone {i+1}: depth={depth:.3f}m')


                    # TODO: Get u, and v of cone in image coordinates
                    u, v = [0. , 0.]

                    # TODO: Find X , Y , Z of cone
                    X = 0.
                    Y = 0.
                    Z = 0. 

                    # Convert to turtlebot frame
                    # There's no camera frame for the turtlebots, so we just do this instead 
                    G = np.array([[0, 0, 1, 0.115],
                      [-1, 0, 0, 0],
                      [0, -1, 0, 0],
                      [0, 0, 0, 1]])
                    goal_point = (G @ np.array([X, Y, Z, 1]).reshape(4, 1)).flatten()

                    point_cam = PointStamped()
                    point_cam.header.stamp = msg.header.stamp
                    point_cam.header.frame_id = 'base_link'
                    point_cam.point.x = goal_point[0]
                    point_cam.point.y = goal_point[1]
                    point_cam.point.z = goal_point[2]
                    self.cone_position_pub.publish(point_cam)
            else:
                self.get_logger().info('No cones spotted')


    def camera_info_callback(self, msg):
        # -------------------------------------------
        # TODO: Extract camera intrinsic parameters! 
        # -------------------------------------------
        self.get_logger().info("Recieved Camera Info")
        

    def visualize_results(self, image, results):
        """
        Visualize YOLO segmentation results using matplotlib.
        Args:
            image: Input image (numpy array)
            results: YOLO results object
        """
        self.ax.clear()
        display_image = image[:, :, ::-1]
        self.ax.imshow(display_image)
        img_height, img_width = image.shape[:2]

        for result in results:
            if result.masks is not None:
                masks = result.masks.data.cpu().numpy()
                self.get_logger().info(f'Drawing {len(masks)} segmentation masks')

                for i, mask in enumerate(masks):
                    self.get_logger().info(f'Mask {i} shape: {mask.shape}, Image shape: {img_height}x{img_width}')

                    if mask.shape != (img_height, img_width):
                        mask_resized = cv2.resize(mask, (img_width, img_height),
                                                 interpolation=cv2.INTER_LINEAR)
                    else:
                        mask_resized = mask

                    bright_green = (0.0, 1.0, 0.0)
                    colored_mask = np.zeros((img_height, img_width, 4))
                    colored_mask[mask_resized > 0.5] = [*bright_green, 0.6]

                    self.ax.imshow(colored_mask, interpolation='nearest')

            boxes = result.boxes
            if boxes is not None and len(boxes) > 0:
                for i, box in enumerate(boxes):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].cpu().numpy()
                    cls = int(box.cls[0].cpu().numpy())

                    width = x2 - x1
                    height = y2 - y1
                    color = plt.cm.tab10(i % 10)[:3]

                    rect = patches.Rectangle(
                        (x1, y1), width, height,
                        linewidth=2, edgecolor=color, facecolor='none'
                    )
                    self.ax.add_patch(rect)

                    label = f'Cone {i+1}: {conf:.2f}'
                    self.ax.text(
                        x1, y1 - 5, label,
                        color='white', fontsize=10, weight='bold',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor=color, alpha=0.7)
                    )

        self.ax.axis('off')
        self.ax.set_title('YOLO Cone Segmentation', fontsize=14, weight='bold')
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
