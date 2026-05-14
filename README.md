# dip
source /opt/ros/jazzy/setup.bash
ros2 launch ros_gz_sim_demos rgbd_camera_bridge.launch.py

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run vision_sorting vision_node --ros-args -p image_topic:=/rgbd_camera/image

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run vision_sorting target_3d_estimator --ros-args \
  -p detections_topic:=/detections \
  -p depth_topic:=/rgbd_camera/depth_image \
  -p camera_info_topic:=/rgbd_camera/camera_info \
  -p target_class:=bottle


ros2 topic echo /target_3d
