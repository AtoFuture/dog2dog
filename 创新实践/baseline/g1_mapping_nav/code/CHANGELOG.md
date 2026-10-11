# Go2 cross-warehouse: code changes (2026-10-07)

## Sim / description
- `src/go2_description/xacro/gazebo_VLP_D435i.xacro`: foot friction mu1/mu2 0.6 -> 1.0
- `src/quadropted_controller/scripts/RobotController/RobotController.py`: default_height 0.25 -> 0.21
- `src/quadropted_controller/scripts/RobotController/TrotGaitController.py`: z_leg_lift 0.14 -> 0.10
- `src/quadropted_controller/scripts/robot_controller_gazebo.py`: IMU subscription + closed-loop roll/pitch compensation (open-loop 25 deg -> 5.5 deg)
- `src/quadropted_controller/scripts/InverseKinematics/robot_IK.py`: sqrt guards (NaN domain errors 8407/min -> 0)
- `src/quadropted_controller/scripts/cmd_vel_pub.py`: cmd_vel scale x 0.12 / y 0.04 (0.3 flips the dog)

## Sim config
- `src/gazebo_sim/config/ekf.yaml`: fixed frames
- `src/gazebo_sim/config/robots.yaml`
- `src/gazebo_sim/launch/launch.py`, `gazebo_go2_sensors.launch.py`
- `src/navigation2/param/go2_nav2.yaml`: velodyne costmap topic fix
- NEW world: `src/gazebo_sim/world/warehouse_cross.sdf` (15 maze walls)

## Maps
- NEW `cross_warehouse_20261007` (1260x1147, occ 11187) - supersedes `cross_warehouse_20260920` (650x304)

## tools/
- `drive_vel.py` / `mode_stick.py` rclpy drivers (ros2 CLI pub hangs under load)
- `swq.py` activate both controllers, `unlq.py`/`lcq.py` controller state repair
- `spawn_robot1.launch.py` rsp + create (z=0.45)
- `map_relay2.py` /map -> /robot1/map relay
- `health.py` / `jsf.py` health & data-flow probes
