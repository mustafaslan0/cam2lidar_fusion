# Fusion of camera and LiDAR data
# author: Mustafa ASLAN
# PARAMETERS 
# cam_topic: Camera image topic
# cam_info_topic: Camera info topic
# lidar_topic: LiDAR point cloud topic
# fused_topic: Fused point cloud topic
# fused_cam_topic: Fused camera image topic
# fused_cam_pub: Publish the fused camera image
# fused_pub: Publish the fused point cloud
# fused_cam_veiw: Display the fused camera image
# camera_positon: Camera position in the world frame
# camera_rpy: Camera orientation in the world frame
# lidar_positon: LiDAR position in the world frame
# lidar_rpy: LiDAR orientation in the world frame
# PUBLISHES
# fused_cam_topic (optional): Fused camera image
# fused_topic (optional): Fused point cloud
# SUBSCRIBES
# cam_topic: Camera image
# cam_info_topic: Camera info
# lidar_topic: LiDAR point cloud
# USAGE
# ros2 run cam2lidar_fusion cam2lidar_fusion
# ros2 run cam2lidar_fusion cam2lidar_fusion --ros-args --params-file src/cam2lidar_fusion/params/fusion.yaml
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2 , PointField,LaserScan
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_transformations
from fusion_interfaces.srv import Distance
from fusion_interfaces.srv import Box
from std_msgs.msg import Header
from laser_geometry import LaserProjection

class Fusion(Node):

    def __init__(self):
        super().__init__('fusion')
        self.bridge = CvBridge()

        self.lidar_mode = self.declare_parameter('lidar_mode', '2d').get_parameter_value().string_value
        self.cam_topic = self.declare_parameter('cam_topic', '/front_camera/image_raw').get_parameter_value().string_value
        self.cam_info_topic = self.declare_parameter('cam_info_topic', '/front_camera/camera_info').get_parameter_value().string_value
        self.lidar_topic = self.declare_parameter('lidar_topic', '/ray/laserscan').get_parameter_value().string_value
        self.fused_topic = self.declare_parameter('fused_topic', '/fused/points').get_parameter_value().string_value
        self.fused_cam_topic = self.declare_parameter('fused_cam_topic', '/fused/camera').get_parameter_value().string_value
        self.fused_cam_pub = self.declare_parameter('fused_cam_pub', True).get_parameter_value().bool_value
        self.fused_pub = self.declare_parameter('fused_pub', True).get_parameter_value().bool_value
        self.fused_cam_veiw = self.declare_parameter('fused_cam_veiw', True).get_parameter_value().bool_value
        self.camera_positon = self.declare_parameter('camera_positon', [0.0, 0.0, -0.49]).get_parameter_value().double_array_value
        self.camera_rpy = self.declare_parameter('camera_rpy', [4.71, 4.71, 3.14]).get_parameter_value().double_array_value
        self.lidar_positon = self.declare_parameter('lidar_positon', [0.0, 0.0, 0.0]).get_parameter_value().double_array_value
        self.lidar_rpy = self.declare_parameter('lidar_rpy', [0.0, 0.0, 0.0]).get_parameter_value().double_array_value


        self.get_logger().info(f'lidar_mode: {self.lidar_mode}')
        self.get_logger().info(f'cam_topic: {self.cam_topic}')
        self.get_logger().info(f'cam_info_topic: {self.cam_info_topic}')
        self.get_logger().info(f'lidar_topic: {self.lidar_topic}')
        self.get_logger().info(f'fused_topic: {self.fused_topic}')
        self.get_logger().info(f'fused_cam_topic: {self.fused_cam_topic}')
        self.get_logger().info(f'fused_cam_pub: {self.fused_cam_pub}')
        self.get_logger().info(f'fused_pub: {self.fused_pub}')
        self.get_logger().info(f'fused_cam_veiw: {self.fused_cam_veiw}')
        

        self.service = self.create_service(Box, 'distance', self.distance_callback)
        self.image_sub = self.create_subscription(Image, self.cam_topic, self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.cam_info_topic, self.camera_info_callback, 10)
        if self.lidar_mode == '3d':
            self.lidar_sub = self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, 10)
        elif self.lidar_mode == '2d':
            self.lidar_sub = self.create_subscription(LaserScan, self.lidar_topic, self.scan_callback, 10)
        else:
            self.get_logger().error('lidar_mode must be 2d or 3d')
            return
        if self.fused_cam_pub:
            self.fused_cam_pub = self.create_publisher(Image, self.fused_cam_topic, 10)
        if self.fused_pub:
            self.fused_pc_pub = self.create_publisher(PointCloud2, self.fused_topic, 10)


        self.camera_matrix = None
        self.dist_coeffs = None
        self.lidar_points = None
        self.camera_image = None
        self.pub_points = []
        self.point_distance_list = []

    def scan_callback(self, msg):
        self.laser_projector = LaserProjection()
        pointcloud_msg = self.laser_projector.projectLaser(msg)
        self.lidar_points = list(pc2.read_points(pointcloud_msg, skip_nans=True, field_names=("x", "y", "z")))

        

   
    def distance_callback(self, request, response):
        self.get_logger().info('Incoming request')
        self.get_logger().info(f'x1: {request.x1} y1: {request.y1} x2: {request.x2} y2: {request.y2}')
        d = 0
        n = 0
        for point in self.point_distance_list:
            if request.x1 < point[0] and point[0] < request.x2 :
                if request.y1 <point[1] and point[1] < request.y2:
                    if point[2] == 0:
                        continue
                    n += 1
                    d += point[2]
                    
        if n > 0:
            response.distance = float(d / n)
            print(f"distance: {d} / {n} = {response.distance}")

            return response
        else:
            response.distance = -1.0
            return response

    def rotation_matrix(self,roll, pitch, yaw):
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])

        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])

        R = np.dot(R_z, np.dot(R_y, R_x))
        return R
    def matrix_calc(self):
        # Kamera ve LiDAR konumları

        camera_pos = self.camera_positon
        camera_rpy = self.camera_rpy

        lidar_pos = self.lidar_positon
        lidar_rpy = self.lidar_rpy

        # Dönüşüm matrisleri
        R_camera = self.rotation_matrix(*camera_rpy)
        R_lidar = self.rotation_matrix(*lidar_rpy)

        # Toplam dönüşüm matrislerini oluşturma
        T_camera = np.eye(4)
        T_camera[:3, :3] = R_camera
        T_camera[:3, 3] = camera_pos

        T_lidar = np.eye(4)
        T_lidar[:3, :3] = R_lidar
        T_lidar[:3, 3] = lidar_pos


        T_camera_to_lidar = np.dot(np.linalg.inv(T_lidar), T_camera)

        return T_camera_to_lidar

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
    
    def image_callback(self, msg):
        self.camera_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.lidar_points is not None and self.camera_matrix is not None:
            self.overlay_lidar_on_image()
        
    def lidar_callback(self, msg):
        self.lidar_points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))

    def overlay_lidar_on_image(self):
        if self.camera_image is None or self.lidar_points is None:
            return
        transformation_matrix = self.matrix_calc()
        
        self.point_distance_list = []
        
        for point in self.lidar_points:
            # LiDAR point in homogeneous coordinates
            lidar_point = np.array([point[0], point[1], point[2], 1])
            
            camera_point =transformation_matrix @ lidar_point

            if camera_point[2] > 0:  # Only consider points in front of the camera
                uv = np.dot(self.camera_matrix, camera_point[:3] / camera_point[2])
                x, y = int(uv[0]), int(uv[1])
                # Draw the point on the image
                if 0 <= x < self.camera_image.shape[1] and 0 <= y < self.camera_image.shape[0]:
                    color = self.camera_image[y, x]
                    pcolor = (color[2] << 16) | (color[1] << 8) | color[0]
                    cv2.circle(self.camera_image, (x, y), 3, (0, 0, 255), -1)
                    self.pub_points.append([point[0], point[1], point[2], pcolor])
                    self.point_distance_list.append([x, y, camera_point[2]])
                    # print(f'x: {x} y: {y} distance: {camera_point[2]}')

        if self.fused_pub:
            self.publish_fused_cloud(self.pub_points)
            self.pub_points = []

                    # print(f'x: {x} y: {y} distance: {camera_point[2]}')

        if self.fused_cam_veiw:
            cv2.imshow("Fused camera lidar image", self.camera_image)
            cv2.waitKey(1)
        if self.fused_cam_pub:
            self.publish_fused_cam(self.camera_image)
        
        
        
        
        

    def publish_fused_cam(self, img):
        img_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        self.fused_cam_pub.publish(img_msg)
    

    def publish_fused_cloud(self, points):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]

        width = len(points)
        point_step = 16  # 4 fields of 4 bytes each (float32)
        row_step = width * point_step
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        
        cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=point_step,
            row_step=row_step,
            data=np.array(points, dtype=np.float32).tobytes()
        )

        self.fused_pc_pub.publish(cloud_msg)
        




def main(args=None):
    rclpy.init(args=args)

    fusion = Fusion()

    rclpy.spin(fusion)

    fusion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()