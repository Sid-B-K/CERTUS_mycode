# CERTUS_mycode

This repository contains code and instructions for working with the [Aslan autonomous driving simulator](https://github.com/project-aslan/Aslan?tab=readme-ov-file), custom worlds, SLAM mapping, Octomap/NDT mapping, and lane detection for a simulated Twizy vehicle.

---

## Aslan Setup

Clone and enter the [Aslan repo](https://github.com/project-aslan/Aslan?tab=readme-ov-file):

```bash
cd Aslan/

# Start the Aslan stack
./run
```

Launch a simulation world:

```bash
# Launch the custom simulation world
roslaunch sd_robot sd_twizy_worlds.launch world:=custom

# Or use a predefined scenario
roslaunch sd_robot sd_twizy_worlds.launch world:=car_1_junction

# View camera stream from the front center camera
/sd_twizy/front_center_camera/image_raw
```

## Worlds

### `harbour world`
<img src="figures/world.png" alt="world" width="50%"/>

---

### `lane junction world`
<img src="figures/laneworld.png" alt="laneworld" width="50%"/>

---

### `cloverleaf track world`
<img src="figures/cloverleaf-junction.png" alt="cloverleaf-junction" width="50%"/>

---

### `mcity world`
<img src="figures/Mcity.png" alt="Mcity" width="50%"/>


## Mapping (SLAM-based) 

Launch required processes to generate a 2D occupancy grid map using GMapping:

```bash
# Launch the simulation with the custom world
roslaunch sd_robot sd_twizy_worlds.launch world:=custom

# Convert pointcloud to 2D laser scan
roslaunch sd_robot pointcloud_to_laserscan.launch

# Launch GMapping for SLAM
roslaunch sd_robot gmapping.launch

```
Control the simulated vehicle:

```bash
# Launch keyboard control
cd /home/sid/Aslan/src/vehicle_simulation/gazebo/sd_twizy_model/streetdrone_model/sd_control
./keyboardlaunch.sh

```
Save and load the map:

```bash
# Save the map to a file
rosrun map_server map_saver -f my_map

# Launch a map server using the saved map
rosrun map_server map_server largemap.yaml

```
Visualize the TF tree:

```bash
rosrun rqt_tf_tree rqt_tf_tree

```

### `NDTmapping`
<p>
  <img src="figures/pc1.png" alt="NDTmapping 1" width="48%" style="margin-right: 5%;">
  <img src="figures/pc2.png" alt="NDTmapping 2" width="35%">
</p>


## Octomap and NDT Mapping (3D Mapping using pointclouds)
Use pointcloud data for volumetric 3D mapping:

```bash
# Start ROS core
roscore

# Publish a PCD file as a pointcloud
rosrun pcl_ros pcd_to_pointcloud /home/sid/Aslan/map.pcd _frame_id:=map _interval:=0.1

# Check available ROS nodes and topics
rosnode list
rostopic echo /cloud_pcd

# Launch Octomap mapping server
roslaunch octomap_server octomap_mapping.launch 

# Save the generated octomap to a file
rosrun octomap_server octomap_saver -f /path/to/save/your_map.ot

# Load an existing Octomap file
rosrun octomap_server octomap_server_node map.ot

```

### `Octomap Views`

<p>
  <img src="figures/octomap.png" alt="Octomap" width="32%" style="margin-right: 2%;">
  <img src="figures/octomap1.png" alt="Octomap 1" width="32%" style="margin-right: 2%;">
  <img src="figures/octomap2.png" alt="Octomap 2" width="32%">
</p>


## Lane Detection
Set up the environment and run lane detection algorithms:

```bash
# Launch the simulation world with junction
roslaunch sd_robot sd_twizy_worlds.launch world:=car_1_junction

# Camera topic to use for lane detection
/sd_twizy/front_center_camera/image_raw

```

Start localizer and transformations:
```bash
# Launch GUI and localization tools
roslaunch aslan_gui baselink_to_localiser.launch

# Static transform between odom and base_link frames
rosrun tf static_transform_publisher 0 0 0 0 0 0 /odom /base_link 10

```

Preprocessing and running lane detection nodes:
```bash
# Convert file format to Unix line endings
dos2unix /home/sid/Aslan/src/vehicle_simulation/gazebo/sd_twizy_model/streetdrone_model/sd_robot/scripts/lane_detection_node.py

# Run lane detection node
rosrun sd_robot lane_detection_node.py

# Run the lane follower and waypoint follower
rosrun sd_robot lane_follower_node.py
rosrun sd_robot waypoint_follower_node.py

```


### `houghtransform lane lines`
<img src="figures/houghtransform.png" alt="Hough Transform" width="50%"/>

---

### `accurate_waypoint detection`
<img src="figures/accurate_waypoint.png" alt="Accurate Waypoint" width="50%"/>

Monitor key topics:
```bash
# Monitor image processing frequency
rostopic hz /lane_detection/image

# Monitor waypoints and control signals
rostopic hz /lane_waypoints
rostopic hz /sd_control

```

## Curve lane detection

<p>
  <img src="figures/curvelanedet.png" alt="Curve Lane Detection 1" width="48%" style="margin-right: 2%;">
  <img src="figures/curvelanedet1.png" alt="Curve Lane Detection 2" width="48%">
</p>

## Notes
* Ensure all necessary dependencies are sourced (source devel/setup.bash).
* Some files may require execution permissions or format conversion (chmod +x, dos2unix).
* Adjust paths if running on a different system or directory structure.
