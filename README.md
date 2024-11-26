# cam2lidar_fusion
The cam2lidar_fusion ROS 2 package provides a solution for fusing data from a camera and LiDAR sensor using a projection-based method, resulting in richer and more meaningful environmental perception. The package utilizes a manually defined transformation matrix to align data from both sensors into a unified coordinate system. This fusion approach enhances object detection and tracking by combining both distance and visual information, making it particularly useful for autonomous vehicles and robotic systems.

#test video

https://youtu.be/DJlzY6ZQrGw

Key features:

- Synchronization of camera and LiDAR data
* Projection method using a manually defined transformation matrix
+ Improved environmental perception and object recognition through data fusion
  
This package is ideal for developers looking to improve the accuracy of systems that rely on both camera and LiDAR for environmental sensing.

# Setup


      mkdir -p colcon_ws/src
      git clone https://github.com/mustafaslan0/cam2lidar_fusion.git
      colcon build
ok now you are ready you can set the parameters in fusion.launch.py and use it
# Example
  
      ros2 run cam2lidar_fusion cam2lidar_fusion
or

      ros2 launch cam2lidar_fusion fusion.launch.py



## Subscribed Topics

  + image_raw (sensor_msgs/Image)
    > raw image topic, for monocular cameras
  + velodyne_points (sensor_msgs/PointCloud2)
    > raw lidar topic

## Published Topics

  + image_raw (sensor_msgs/Image)
    >/fused/camera

## Parameters
  + lidar_mode: '2d' or '3d'
    >lidar type 2d and 3d 
  + cam_topic: '/front_camera/image_raw'
    >subscribed camera
  + cam_info_topic: '/front_camera/camera_info'
    >with this topic the camera is calibrated
  + lidar_topic: '/cloud' or 'laser_scan'
    >subscribed lidar topic
  + fused_topic: '/fused/points'
    >fused pointcloud
  + fused_cam_topic: '/fused/camera'
    > Published camera topic
  + fused_cam_pub: True
    >setting to publish camera
  + fused_pub: False
    >setting to publish pointcloud2
  + fused_cam_veiw: True
    > the camera image is visible on the screen
  + camera_positon: [0.0, 0.0, -0.49]
  + camera_rpy: [4.71, 4.71, 3.14] 
  + lidar_positon: [0.0, 0.0, 0.0]
  + lidar_rpy: [0.0, 0.0, 0.0]
     > for the trasform matrix, the position and angle information of the camera and lidar must be entered



![image](https://github.com/user-attachments/assets/df7ee5cd-9448-4c80-ad04-cb9450c62358)

![image](https://github.com/user-attachments/assets/3a86ff0a-7dbd-4468-a639-642ed8816647)


                
                
               



      
      

      

      

  
