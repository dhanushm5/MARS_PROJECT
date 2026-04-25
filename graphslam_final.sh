#!/bin/bash
# ================================================================
#  GraphSLAM FINAL - Turtlebot3 Waffle Equivalent + RGB-D + Lidar
#  WITH: Automated Before/After Loop Closure Map Capture
# ================================================================

export DISPLAY=:1
export XAUTHORITY=/home/ubuntu/.Xauthority
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
source /opt/ros/humble/setup.bash 2>/dev/null || { echo "ERROR: ROS2 not found"; exit 1; }

WS=$HOME/graphslam_ws
LOG=$WS/run.log
RESULTS=$WS/results
MODEL_DIR=$WS/models/tb3_waffle_sim
export IGN_GAZEBO_RESOURCE_PATH=$WS/models

# Flag files used to signal the map snapshot node
SNAP_BEFORE_FLAG=$RESULTS/.snap_before
SNAP_AFTER_FLAG=$RESULTS/.snap_after
SNAP_DONE_FLAG=$RESULTS/.snap_done

mkdir -p $MODEL_DIR $WS/worlds $RESULTS
> $LOG
rm -f $SNAP_BEFORE_FLAG $SNAP_AFTER_FLAG $SNAP_DONE_FLAG

cleanup() {
  pkill -9 -f "ign|rtabmap|rviz2|ros_gz|static_transform|icp_odometry|traj_rec|auto_driver|map_snapshot" 2>/dev/null
}
trap cleanup EXIT INT TERM
cleanup; sleep 2

echo "================================================================"
echo "  GraphSLAM Demo (RGB-D + Lidar) - With Loop Closure Capture"
echo "================================================================"

pip install --break-system-packages -q "numpy<2" matplotlib 2>/dev/null || true

# ════════════════════════════════════════════════════════════════════════════════
# WRITE RVIZ2 CONFIG
# ════════════════════════════════════════════════════════════════════════════════
cat > $WS/graphslam.rviz << 'RVIZ_EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Name: OccupancyMap
      Topic:
        Value: /rtabmap/grid_map
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Name: MapGraph
      Topic:
        Value: /rtabmap/mapGraph
      Value: true
    - Alpha: 1.0
      Autocompute Intensity Bounds: true
      Class: rviz_default_plugins/PointCloud2
      Name: CloudMap (3D RGB-D)
      Topic:
        Value: /rtabmap/cloud_map
      Value: true
      Size (m): 0.05
    - Alpha: 1.0
      Class: rviz_default_plugins/LaserScan
      Name: LaserScan
      Topic:
        Value: /scan
      Value: true
      Size (m): 0.05
      Color: 255; 0; 0
    - Class: rviz_default_plugins/TF
      Name: TF
      Value: true
      Show Arrows: false
      Show Axes: true
      Show Names: false
      Frame Timeout: 15
    - Class: rviz_default_plugins/Odometry
      Name: Odometry
      Topic:
        Value: /odom
      Value: true
      Keep: 500
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: TopDownOrtho
      Scale: 40
      Target Frame: <Fixed Frame>
      X: 0
      Y: 0
RVIZ_EOF

# ════════════════════════════════════════════════════════════════════════════════
# WRITE WORLD
# ════════════════════════════════════════════════════════════════════════════════
cat > $WS/worlds/world.sdf << 'WORLD_EOF'
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="indoor">
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose><diffuse>1 1 1 1</diffuse><direction>-0.5 0.1 -0.9</direction>
    </light>
    <light name="fill" type="point">
      <pose>0 0 5 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse>
      <attenuation><range>30</range><linear>0.05</linear></attenuation>
    </light>
    <model name="ground"><static>true</static><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
        <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
    </link></model>
    <model name="wN"><static>true</static><pose>0 8 1 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>16 0.2 2</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>16 0.2 2</size></box></geometry><material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual></link></model>
    <model name="wS"><static>true</static><pose>0 -8 1 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>16 0.2 2</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>16 0.2 2</size></box></geometry><material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual></link></model>
    <model name="wE"><static>true</static><pose>8 0 1 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>0.2 16 2</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.2 16 2</size></box></geometry><material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual></link></model>
    <model name="wW"><static>true</static><pose>-8 0 1 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>0.2 16 2</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.2 16 2</size></box></geometry><material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual></link></model>
    <model name="iwHL"><static>true</static><pose>-5.5 0 1 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>4 0.2 2</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 0.2 2</size></box></geometry><material><ambient>0.65 0.65 0.5 1</ambient><diffuse>0.65 0.65 0.5 1</diffuse></material></visual></link></model>
    <model name="iwHR"><static>true</static><pose>5.5 0 1 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>4 0.2 2</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 0.2 2</size></box></geometry><material><ambient>0.65 0.65 0.5 1</ambient><diffuse>0.65 0.65 0.5 1</diffuse></material></visual></link></model>
    <model name="iwVT"><static>true</static><pose>0 5.5 1 0 0 1.5708</pose><link name="l">
      <collision name="c"><geometry><box><size>4 0.2 2</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 0.2 2</size></box></geometry><material><ambient>0.65 0.65 0.5 1</ambient><diffuse>0.65 0.65 0.5 1</diffuse></material></visual></link></model>
    <model name="iwVB"><static>true</static><pose>0 -5.5 1 0 0 1.5708</pose><link name="l">
      <collision name="c"><geometry><box><size>4 0.2 2</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>4 0.2 2</size></box></geometry><material><ambient>0.65 0.65 0.5 1</ambient><diffuse>0.65 0.65 0.5 1</diffuse></material></visual></link></model>
    <model name="red"><static>true</static><pose>-6 6.5 0.5 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>0.8 0.8 1</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.8 0.8 1</size></box></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link></model>
    <model name="green"><static>true</static><pose>6 6.5 0.5 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>0.8 0.8 1</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.8 0.8 1</size></box></geometry><material><ambient>0 0.8 0 1</ambient><diffuse>0 0.8 0 1</diffuse></material></visual></link></model>
    <model name="blue"><static>true</static><pose>6 -6.5 0.5 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>0.8 0.8 1</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.8 0.8 1</size></box></geometry><material><ambient>0 0 1 1</ambient><diffuse>0 0 1 1</diffuse></material></visual></link></model>
    <model name="yellow"><static>true</static><pose>-6 -6.5 0.5 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>0.8 0.8 1</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.8 0.8 1</size></box></geometry><material><ambient>1 0.9 0 1</ambient><diffuse>1 0.9 0 1</diffuse></material></visual></link></model>
    <include>
      <uri>model://tb3_waffle_sim</uri>
      <pose>0 0 0.05 0 0 0</pose>
    </include>
  </world>
</sdf>
WORLD_EOF

# ════════════════════════════════════════════════════════════════════════════════
# WRITE ROBOT MODEL
# ════════════════════════════════════════════════════════════════════════════════
cat > $MODEL_DIR/model.config << 'EOF'
<?xml version="1.0"?>
<model><n>tb3_waffle_sim</n><version>1.0</version><sdf version="1.8">model.sdf</sdf></model>
EOF

cat > $MODEL_DIR/model.sdf << 'SDF_EOF'
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="tb3_waffle_sim">
    <link name="base_footprint">
      <inertial><mass>0.001</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial>
    </link>
    <joint name="base_joint" type="fixed"><parent>base_footprint</parent><child>base_link</child></joint>
    <link name="base_link">
      <pose>0 0 0.08 0 0 0</pose>
      <inertial><mass>2.0</mass><inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.02</izz></inertia></inertial>
      <collision name="c"><geometry><box><size>0.26 0.26 0.12</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.26 0.26 0.12</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
    </link>
    <link name="wheel_left_link">
      <pose>0 0.144 0.033 0 0 0</pose>
      <inertial><mass>0.3</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>
      <collision name="c"><pose>0 0 0 1.5707963 0 0</pose>
        <geometry><cylinder><radius>0.033</radius><length>0.018</length></cylinder></geometry>
        <surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction></surface></collision>
      <visual name="v"><pose>0 0 0 1.5707963 0 0</pose>
        <geometry><cylinder><radius>0.033</radius><length>0.018</length></cylinder></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
    </link>
    <joint name="wheel_left_joint" type="revolute">
      <parent>base_link</parent><child>wheel_left_link</child>
      <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.1</damping><friction>0.1</friction></dynamics></axis>
    </joint>
    <link name="wheel_right_link">
      <pose>0 -0.144 0.033 0 0 0</pose>
      <inertial><mass>0.3</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>
      <collision name="c"><pose>0 0 0 1.5707963 0 0</pose>
        <geometry><cylinder><radius>0.033</radius><length>0.018</length></cylinder></geometry>
        <surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction></surface></collision>
      <visual name="v"><pose>0 0 0 1.5707963 0 0</pose>
        <geometry><cylinder><radius>0.033</radius><length>0.018</length></cylinder></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
    </link>
    <joint name="wheel_right_joint" type="revolute">
      <parent>base_link</parent><child>wheel_right_link</child>
      <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.1</damping><friction>0.1</friction></dynamics></axis>
    </joint>
    <link name="caster_link">
      <pose>-0.1 0 0.018 0 0 0</pose><inertial><mass>0.05</mass></inertial>
      <collision name="c"><geometry><sphere><radius>0.018</radius></sphere></geometry>
        <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>
    </link>
    <joint name="caster_joint" type="ball"><parent>base_link</parent><child>caster_link</child></joint>
    <link name="lidar_link">
      <pose>0 0 0.2 0 0 0</pose>
      <visual name="v"><geometry><cylinder><radius>0.035</radius><length>0.04</length></cylinder></geometry>
        <material><ambient>0.9 0.1 0.1 1</ambient></material></visual>
      <sensor name="lidar" type="gpu_lidar">
        <always_on>true</always_on><update_rate>10</update_rate>
        <topic>/scan</topic><gz_frame_id>lidar_link</gz_frame_id>
        <lidar>
          <scan><horizontal><samples>360</samples><resolution>1</resolution>
            <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>
          <range><min>0.12</min><max>12.0</max><resolution>0.01</resolution></range>
        </lidar>
      </sensor>
    </link>
    <joint name="lidar_joint" type="fixed"><parent>base_link</parent><child>lidar_link</child></joint>
    <link name="camera_link">
      <pose>0.064 0 0.094 0 0 0</pose>
      <visual name="v">
        <geometry><box><size>0.015 0.130 0.022</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient></material>
      </visual>
      <sensor name="camera" type="rgbd_camera">
        <always_on>true</always_on>
        <update_rate>15</update_rate>
        <topic>/camera</topic>
        <gz_frame_id>camera_link_optical</gz_frame_id>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image><width>320</width><height>240</height></image>
          <clip><near>0.1</near><far>10</far></clip>
        </camera>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed"><parent>base_link</parent><child>camera_link</child></joint>
    <plugin filename="libignition-gazebo-diff-drive-system.so"
            name="ignition::gazebo::systems::DiffDrive">
      <left_joint>wheel_left_joint</left_joint><right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.288</wheel_separation><wheel_radius>0.033</wheel_radius>
      <odom_publish_frequency>30</odom_publish_frequency>
      <topic>/cmd_vel</topic><odom_topic>/odom</odom_topic>
      <tf_topic>/tf</tf_topic><frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
      <min_acceleration>-10</min_acceleration><max_acceleration>10</max_acceleration>
    </plugin>
    <plugin filename="libignition-gazebo-joint-state-publisher-system.so"
            name="ignition::gazebo::systems::JointStatePublisher"/>
  </model>
</sdf>
SDF_EOF

# ════════════════════════════════════════════════════════════════════════════════
# 1. GAZEBO
# ════════════════════════════════════════════════════════════════════════════════
echo "[1/6] Starting Gazebo..."
ign gazebo -s -r $WS/worlds/world.sdf >> $LOG 2>&1 &
echo -n "      Waiting for Gazebo "
for i in $(seq 1 25); do
  sleep 1; echo -n "."
  ign topic -l 2>/dev/null | grep -q "^/odom$" && echo " ✓" && break
done
ign gazebo -g >> $LOG 2>&1 &
sleep 4

# ════════════════════════════════════════════════════════════════════════════════
# 2. BRIDGE + TF
# ════════════════════════════════════════════════════════════════════════════════
echo "[2/6] Starting bridge for Sensors and Clock..."
ros2 run ros_gz_bridge parameter_bridge \
  /scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
  /odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry \
  /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
  /tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V \
  /camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo \
  /camera/image@sensor_msgs/msg/Image[ignition.msgs.Image \
  /camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image \
  /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock \
  >> $LOG 2>&1 &

sleep 4

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link --ros-args -p use_sim_time:=true >> $LOG 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0.2 0 0 0 base_link lidar_link --ros-args -p use_sim_time:=true >> $LOG 2>&1 &
ros2 run tf2_ros static_transform_publisher 0.064 0 0.094 0 0 0 base_link camera_link --ros-args -p use_sim_time:=true >> $LOG 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 camera_link camera_link_optical --ros-args -p use_sim_time:=true >> $LOG 2>&1 &
sleep 2

# ════════════════════════════════════════════════════════════════════════════════
# 3. RTABMAP
# ════════════════════════════════════════════════════════════════════════════════
echo "[3/6] Launching RTABMap (RGB-D + Lidar Mode)..."
ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start --Grid/Sensor 0" \
  icp_odometry:=false \
  visual_odometry:=false \
  odom_topic:=/odom \
  subscribe_rgb:=true \
  subscribe_depth:=true \
  subscribe_scan:=true \
  rgb_topic:=/camera/image \
  depth_topic:=/camera/depth_image \
  camera_info_topic:=/camera/camera_info \
  scan_topic:=/scan \
  frame_id:=base_footprint \
  odom_frame_id:=odom \
  approx_sync:=true \
  queue_size:=50 \
  use_sim_time:=true \
  rviz:=false \
  >> $LOG 2>&1 &

sleep 10

# ════════════════════════════════════════════════════════════════════════════════
# 4. RVIZ2
# ════════════════════════════════════════════════════════════════════════════════
echo "[4/6] Launching RViz2..."
ros2 run rviz2 rviz2 -d $WS/graphslam.rviz --ros-args -p use_sim_time:=true >> $LOG 2>&1 &
sleep 5

# ════════════════════════════════════════════════════════════════════════════════
# 5. MAP SNAPSHOT NODE (Fixed QoS Profile)
# ════════════════════════════════════════════════════════════════════════════════
echo "[5/6] Starting map snapshot node..."

python3 - << 'SNAP_EOF' >> $LOG 2>&1 &
import rclpy, os, sys
import numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data # <-- FIX: Accepts any Durability

RESULTS = os.path.expanduser('~/graphslam_ws/results')
SNAP_BEFORE = os.path.join(RESULTS, '.snap_before')
SNAP_AFTER  = os.path.join(RESULTS, '.snap_after')
SNAP_DONE   = os.path.join(RESULTS, '.snap_done')

def occupancy_to_image(grid_msg):
    w = grid_msg.info.width
    h = grid_msg.info.height
    data = np.array(grid_msg.data, dtype=np.int8).reshape(h, w)
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[data == -1]  = [128, 128, 128]   
    img[data == 0]   = [240, 240, 240]   
    img[data > 0]    = [20,  20,  20]    
    return np.flipud(img)

def save_map_png(img_array, path, label):
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(img_array, interpolation='nearest')
    ax.set_title(label, fontsize=14, fontweight='bold', color='white', pad=10)
    ax.axis('off')
    fig.patch.set_facecolor('#1a1a2e')
    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    plt.close()

def make_comparison(before_img, after_img, out_path):
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(20, 10))
    fig.patch.set_facecolor('#0a0a1a')
    titles = ['BEFORE Loop Closure\n(Accumulated Odometry Drift)', 'AFTER Loop Closure\n(Graph-Optimized Map)']
    images = [before_img, after_img]
    border_colors = ['#ff4444', '#44ff44']

    for ax, img, title, color in zip(axes, images, titles, border_colors):
        ax.imshow(img, interpolation='nearest')
        ax.set_title(title, fontsize=15, fontweight='bold', color=color, pad=12)
        ax.axis('off')
        for spine in ax.spines.values():
            spine.set_edgecolor(color)
            spine.set_linewidth(3)
            spine.set_visible(True)

    fig.text(0.5, 0.02, '← Drift visible in wall alignment         Loop closure detected → pose graph corrected →', ha='center', color='#aaaaaa', fontsize=11, style='italic')
    fig.suptitle('GraphSLAM Loop Closure: Before vs After Optimization', fontsize=17, fontweight='bold', color='white', y=1.01)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight', facecolor='#0a0a1a')
    plt.close()

class MapSnapshotNode(Node):
    def __init__(self):
        super().__init__('map_snapshot', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.latest_grid = None
        self.before_img  = None
        self.after_img   = None
        self.did_before  = False
        self.did_after   = False
        self.after_trigger_time = 0.0
        
        # <-- FIX: Use standard sensor data profile so it isn't strict on Transient Local
        self.create_subscription(OccupancyGrid, '/rtabmap/grid_map', self._map_cb, qos_profile_sensor_data)
        
        self.create_timer(0.5, self._check_flags)

    def _map_cb(self, msg):
        self.latest_grid = msg

    def _check_flags(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if not self.did_before and os.path.exists(SNAP_BEFORE):
            if self.latest_grid is not None:
                self.before_img = occupancy_to_image(self.latest_grid)
                save_map_png(self.before_img, os.path.join(RESULTS, 'map_before_loop_closure.png'), 'Map BEFORE Loop Closure')
                self.did_before = True

        if not self.did_after and os.path.exists(SNAP_AFTER):
            if self.after_trigger_time == 0.0:
                self.after_trigger_time = now
            elif (now - self.after_trigger_time) > 4.0: 
                if self.latest_grid is not None:
                    self.after_img = occupancy_to_image(self.latest_grid)
                    save_map_png(self.after_img, os.path.join(RESULTS, 'map_after_loop_closure.png'), 'Map AFTER Loop Closure')
                    self.did_after = True

        if self.did_before and self.did_after and not os.path.exists(SNAP_DONE):
            if self.before_img is not None and self.after_img is not None:
                make_comparison(self.before_img, self.after_img, os.path.join(RESULTS, 'loop_closure_comparison.png'))
            open(SNAP_DONE, 'w').close()

rclpy.init()
node = MapSnapshotNode()
try: rclpy.spin(node)
except: pass
finally: node.destroy_node(); rclpy.shutdown()
SNAP_EOF

SNAPSHOT_PID=$!
sleep 3

# ════════════════════════════════════════════════════════════════════════════════
# 6. TRAJECTORY RECORDER + AUTO DRIVE
# ════════════════════════════════════════════════════════════════════════════════
echo "[6/6] Starting auto-navigation..."

# Trajectory recorder
python3 - << 'PYEOF' >> $LOG 2>&1 &
import rclpy, json, os
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry

poses = []
OUT = os.path.expanduser('~/graphslam_ws/results/trajectory_raw.json')

class Rec(Node):
    def __init__(self):
        super().__init__('traj_rec', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.create_subscription(Odometry, '/odom', self.cb, 10)
    def cb(self, msg):
        poses.append({'x': round(msg.pose.pose.position.x,3), 'y': round(msg.pose.pose.position.y,3)})
        if len(poses) % 50 == 0:
            with open(OUT,'w') as f: json.dump(poses, f)

rclpy.init()
n = Rec()
try: rclpy.spin(n)
except: pass
with open(OUT,'w') as f: json.dump(poses, f)
PYEOF
RECORDER_PID=$!
sleep 2

echo "================================================================"
echo "  Sensors Active: 2D LaserScan + RGB-D Camera"
echo "  Map snapshots will be captured automatically:"
echo "    • Before: just before return-to-origin"
echo "    • After:  after loop closure correction"
echo "  Output: results/loop_closure_comparison.png"
echo "================================================================"

# Auto driver
python3 - << 'DRIVE_EOF'
import rclpy, math, time, os
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist

RESULTS        = os.path.expanduser('~/graphslam_ws/results')
SNAP_BEFORE    = os.path.join(RESULTS, '.snap_before')
SNAP_AFTER     = os.path.join(RESULTS, '.snap_after')

class Driver(Node):
    def __init__(self):
        super().__init__('auto_driver', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def get_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def cmd(self, v, w):
        t = Twist(); t.linear.x = float(v); t.angular.z = float(w)
        self.pub.publish(t)

    def stop(self, secs=0.5):
        end = self.get_time() + secs
        while self.get_time() < end:
            self.cmd(0, 0); rclpy.spin_once(self, timeout_sec=0.05)

    def forward(self, dist, speed=0.4):
        if dist == 0: return
        secs = dist / speed
        end = self.get_time() + secs
        while self.get_time() < end:
            self.cmd(speed, 0); rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()

    def turn(self, deg, speed=0.5):
        if deg == 0: return
        dur = abs(math.radians(deg)) / speed
        end = self.get_time() + dur
        while self.get_time() < end:
            self.cmd(0, (1 if deg > 0 else -1) * speed)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()

    def scan_room(self):
        self.turn(360, speed=0.8)
        self.stop(0.5)

    def run(self):
        self.get_logger().info('Waiting 8s for clock sync...')
        time.sleep(8)

        while self.get_time() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Starting full map exploration...')
        self.scan_room()

        self.turn(90);  self.forward(2.5)
        self.turn(90);  self.forward(6.0)
        self.turn(-90); self.forward(4.0)
        self.scan_room()

        self.turn(180); self.forward(4.0)
        self.turn(90);  self.forward(12.0)
        self.turn(90);  self.forward(4.0)
        self.scan_room()

        self.turn(180); self.forward(4.0)
        self.turn(-90); self.forward(6.0)
        self.turn(90);  self.forward(5.0)
        self.turn(90);  self.forward(6.0)
        self.turn(-90); self.forward(4.0)
        self.scan_room()

        self.turn(180); self.forward(4.0)
        self.turn(90);  self.forward(12.0)
        self.turn(90);  self.forward(4.0)
        self.scan_room()

        self.get_logger().info('=== TRIGGERING BEFORE-SNAPSHOT ===')
        open(SNAP_BEFORE, 'w').close()
        self.stop(2.0)   

        self.get_logger().info('=== RETURNING TO ORIGIN — LOOP CLOSURE! ===')
        self.turn(180); self.forward(4.0)   
        self.turn(-90); self.forward(6.0)   
        self.turn(90);  self.forward(2.5)   
        self.scan_room()
        self.stop(5)

        self.get_logger().info('=== TRIGGERING AFTER-SNAPSHOT ===')
        open(SNAP_AFTER, 'w').close()
        self.stop(5.0)   

rclpy.init()
d = Driver()
try:
    d.run()
except Exception as e:
    d.get_logger().error(f'Error: {e}')
finally:
    d.stop(2); d.destroy_node(); rclpy.shutdown()
DRIVE_EOF

# ════════════════════════════════════════════════════════════════════════════════
# SAVE RESULTS (Fixed nav2 parameters for Sim Time and Topic Namespace)
# ════════════════════════════════════════════════════════════════════════════════
# ════════════════════════════════════════════════════════════════════════════════
# SAVE RESULTS
# ════════════════════════════════════════════════════════════════════════════════
echo ""
echo "[SAVING] Navigation complete. Saving all results..."

# Wait up to 30 s for the comparison image to be written
echo -n "         Waiting for loop closure comparison "
for i in $(seq 1 30); do
  sleep 1; echo -n "."
  [ -f "$RESULTS/.snap_done" ] && echo " ✓" && break
done

kill $RECORDER_PID $SNAPSHOT_PID 2>/dev/null; sleep 2

# 1. SAVE THE 2D GRID MAP (Replaces nav2_map_server)
echo "[SAVE] Extracting 2D Grid Map..."
python3 - << 'MAP_EOF'
import rclpy, sys, os, struct, time, threading
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        self.map_saved = False
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos)

    def map_cb(self, msg):
        if self.map_saved: return
        self.map_saved = True
        R = os.path.expanduser('~/graphslam_ws/results')
        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        
        with open(f'{R}/graphslam_map.pgm', 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode())
            for row in range(h-1, -1, -1):
                for col in range(w):
                    val = msg.data[row * w + col]
                    if val == -1:
                        pixel = 205      # Unknown space (grey)
                    elif val <= 25:
                        pixel = 254      # Free space (white)
                    elif val >= 65:
                        pixel = 0        # Solid wall/obstacle (black)
                    else:
                        pixel = 205      # Uncertain/noise (grey)
                        
                    f.write(struct.pack('B', pixel))
                    
        with open(f'{R}/graphslam_map.yaml', 'w') as f:
            f.write(f"image: graphslam_map.pgm\nresolution: {res}\norigin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

rclpy.init()
node = MapSaver()
timeout = time.time() + 10
while not node.map_saved and time.time() < timeout:
    rclpy.spin_once(node, timeout_sec=0.1)

if node.map_saved: print("[SAVE] ✓ 2D Grid Map and YAML saved")
else: print("[SAVE] ✗ Map save failed (Timeout)")
node.destroy_node(); rclpy.shutdown()
MAP_EOF

# 2. SAVE THE RTABMAP DATABASE
cp ~/.ros/rtabmap.db $RESULTS/rtabmap_final.db 2>/dev/null && \
  echo "[SAVE] ✓ RTABMap DB saved"


# Plot
python3 - << 'PLOT_EOF'
import json, os, sys
try:
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt, matplotlib.patches as patches
except: sys.exit(0)

R = os.path.expanduser('~/graphslam_ws/results')
poses = json.load(open(f'{R}/trajectory_raw.json')) if os.path.exists(f'{R}/trajectory_raw.json') else []
if len(poses) < 5: sys.exit(0)

xs=[p['x'] for p in poses]; ys=[p['y'] for p in poses]; n=len(xs)
fig,ax = plt.subplots(figsize=(12,12))
ax.set_facecolor('#0a0a1a'); fig.patch.set_facecolor('#0a0a1a')
for x,y,w,h in [(-8,-8,16,.2),(-8,7.8,16,.2),(-8,-8,.2,16),(7.8,-8,.2,16)]:
    ax.add_patch(patches.Rectangle((x,y),w,h,color='#999'))
for x,y,w,h in [(-7.5,-.1,4,.2),(3.5,-.1,4,.2),(-.1,3.5,.2,4),(-.1,-7.5,.2,4)]:
    ax.add_patch(patches.Rectangle((x,y),w,h,color='#776'))
for rx,ry,nm,col in [(-6,6.5,'Room1\n(Red)','#f44'),(6,6.5,'Room2\n(Green)','#4f4'),(6,-6.5,'Room3\n(Blue)','#44f'),(-6,-6.5,'Room4\n(Yellow)','#ff4')]:
    ax.add_patch(patches.Rectangle((rx-.4,ry-.5),.8,1,color=col,zorder=5,alpha=.8))
    ax.text(rx,ry+1.3,nm,color=col,fontsize=9,ha='center',fontweight='bold')
for i in range(n-1):
    t=i/n; ax.plot([xs[i],xs[i+1]],[ys[i],ys[i+1]],'-',color=(t,.4,1-t),lw=2.5,alpha=.9)
ax.scatter(xs[0],ys[0],c='lime',s=400,zorder=10,marker='*',label='Start')
ax.scatter(xs[-1],ys[-1],c='red',s=250,zorder=10,marker='X',label='End')
dist=sum(((xs[i+1]-xs[i])**2+(ys[i+1]-ys[i])**2)**.5 for i in range(n-1))
ax.set_title('GraphSLAM Trajectory (RGB-D + Lidar)\nLoop closure on return to origin',color='white',fontsize=13,fontweight='bold')
ax.set_xlabel('X (m)',color='white'); ax.set_ylabel('Y (m)',color='white')
ax.tick_params(colors='white'); ax.grid(True,alpha=.15,color='white')
ax.legend(loc='lower right',facecolor='#1a1a2e',labelcolor='white')
ax.set_aspect('equal'); ax.set_xlim(-9,9); ax.set_ylim(-9,9)
ax.text(.02,.02,f'Poses:{n} Dist:{dist:.1f}m',transform=ax.transAxes,color='gray',fontsize=8)
[s.set_color('#444') for s in ax.spines.values()]
out=f'{R}/trajectory_plot.png'
plt.tight_layout(); plt.savefig(out,dpi=150,bbox_inches='tight',facecolor='#0a0a1a')
print(f'[PLOT] Trajectory saved: {out}')
PLOT_EOF

echo "================================================================"
echo "  ALL DONE — Results in ~/graphslam_ws/results/"
echo ""
echo "  map_before_loop_closure.png   ← drift visible"
echo "  map_after_loop_closure.png    ← corrected by graph optimisation"
echo "  loop_closure_comparison.png   ← side-by-side for your report"
echo "  trajectory_plot.png           ← colour-coded path"
echo "  graphslam_map.pgm/.yaml       ← ROS 2 nav map"
echo "  rtabmap_final.db              ← full RTAB-Map database"
echo "================================================================"
wait