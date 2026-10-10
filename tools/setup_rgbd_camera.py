#!/usr/bin/env python3
"""
setup_rgbd_camera.py -- step 1 of VIO: give the robot an RGB-D camera and bridge it to ROS.

URDF (src/my_robot/urdf/robot.urdf.xacro):
  - camera becomes an rgbd_camera: colour + depth, 640x480 at 20 Hz, depth 0.1-10 m
  - new frame camera_optical_link (z forward, x right, y down), used as the images' frame_id
  - body bright blue, LiDAR orange (easier to see in the video)
Launch (src/my_robot/launch/gazebo_ekf.launch.py):
  - second bridge 'camera_bridge': /camera/image, /camera/depth_image, /camera/camera_info

Backups on first run: *.prergbd.bak. Run from ~/autonomy:  python3 tools/setup_rgbd_camera.py
"""
import os
import re
import shutil

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(ROOT, "src", "my_robot", "urdf", "robot.urdf.xacro")
LAUNCH = os.path.join(ROOT, "src", "my_robot", "launch", "gazebo_ekf.launch.py")


def backup(p):
    b = p + ".prergbd.bak"
    if not os.path.exists(b):
        shutil.copyfile(p, b)
        print("backup:", b)


def must_replace(s, old, new, what):
    if old not in s:
        raise SystemExit("could not find %s, nothing written. Send this message to Claude." % what)
    return s.replace(old, new)


# ------------------------------------------------------------------ URDF
s = open(URDF).read().replace("\r\n", "\n")
if "camera_optical_link" in s:
    print("URDF: already patched")
else:
    backup(URDF)
    s = must_replace(s, '<material name="blue"><color rgba="0.2 0.3 0.8 1.0"/></material>',
                     '<material name="blue"><color rgba="0.0 0.35 1.0 1.0"/></material>\n'
                     '  <material name="orange"><color rgba="1.0 0.5 0.0 1.0"/></material>', "blue material")
    s = must_replace(s, '<visual><geometry><cylinder radius="0.04" length="0.04"/></geometry><material name="grey"/></visual>',
                     '<visual><geometry><cylinder radius="0.04" length="0.04"/></geometry><material name="orange"/></visual>',
                     "lidar visual")
    j = s.index('  <joint name="camera_joint" type="fixed">')
    j = s.index("</joint>", j) + len("</joint>\n")
    s = s[:j] + '''
  <!-- Optical frame for the RGB-D images (z forward, x right, y down) -->
  <link name="camera_optical_link"/>
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_link"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>
''' + s[j:]
    sensor = '''<sensor name="camera" type="rgbd_camera">
      <topic>camera</topic>
      <gz_frame_id>camera_optical_link</gz_frame_id>
      <update_rate>20</update_rate>
      <always_on>1</always_on>
      <visualize>false</visualize>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>20</far>
        </clip>
        <depth_camera>
          <clip>
            <near>0.1</near>
            <far>10</far>
          </clip>
        </depth_camera>
      </camera>
    </sensor>'''
    s, n = re.subn(r'<sensor name="camera" type="camera">.*?</sensor>', sensor, s, flags=re.S)
    if n != 1:
        raise SystemExit("could not find the camera sensor block, nothing written. Send this message to Claude.")
    open(URDF, "w").write(s)
    print("URDF: rgbd_camera 640x480 @ 20 Hz, camera_optical_link, blue body, orange LiDAR")

# ------------------------------------------------------------------ launch
s = open(LAUNCH).read()
if "camera_bridge" in s:
    print("launch: already patched")
else:
    backup(LAUNCH)
    block = '''    # RGB-D camera topics for visual odometry (rtabmap rgbd_odometry)
    camera_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription(['''
    s = must_replace(s, "    return LaunchDescription([", block, "return LaunchDescription")
    s = must_replace(s, "spawn, bridge,", "spawn, bridge, camera_bridge,", "'spawn, bridge,' in the launch list")
    open(LAUNCH, "w").write(s)
    print("launch: camera_bridge added (/camera/image, /camera/depth_image, /camera/camera_info)")
print("next: colcon build --symlink-install, then start gazebo_ekf.launch.py")