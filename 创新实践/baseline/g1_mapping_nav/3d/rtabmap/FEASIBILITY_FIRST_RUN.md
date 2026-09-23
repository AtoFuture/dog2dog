# RTAB-Map feasibility + first-run (Go2 sim, 2026-09-20)
## Feasibility
- RGB-D topics OK (passive only, no cmd_vel):
  /robot1/color/image_raw ~2.5Hz (frame camera_face, 640x480 rgb8)
  /robot1/color/camera_info ~3.0Hz (frame camera_face, 640x480, K fx~528)
  /robot1/rgbd_d435/image ~8-9Hz (frame camera_d435, 640x480 rgb8)
  /robot1/rgbd_d435/depth_image ~8-9Hz (frame camera_d435, 640x480 32FC1 meters)
  /robot1/odometry/filtered ~9Hz
- NOTE abnormal: NO /robot1/rgbd_d435/camera_info exists (only /robot1/color/camera_info).
  Trial-1 workaround: rgb=/robot1/rgbd_d435/image + depth=/robot1/rgbd_d435/depth_image + info=/robot1/color/camera_info, approx_sync true.
  Frame mismatch camera_d435 vs camera_face tolerated via static TF base_link->camera_*; rate mismatch 9Hz vs 3Hz tolerated via approx_sync_max_interval 0.5s.
  Next: use d435_camera_info_relay.py to republish color info as /robot1/rgbd_d435/camera_info with frame_id=camera_d435 for clean sync.
- TF: sim publishes namespaced /robot1/tf + /robot1/tf_static (odom->base_link, base_link->camera_face/camera_d435/trunk/lidar...; map->odom via amcl).
  Launch remaps /tf->/robot1/tf, /tf_static->/robot1/tf_static. publish_tf=false to avoid fighting amcl.
- Odom: rtabmap uses TF odom (odom_frame_id=odom set => subscribe_odom forced false). Equivalent to filtered odom topic; OK for first run.
- Disk/mem: / 7.0T total, 379G avail (95% used), mem 125G. Space OK.
- Package: ros-humble-rtabmap-ros was NOT installed; apt candidate 0.23.7 available. Installed OK via --root enroot with /tmp/apt.lock.
## Install
- ros-humble-rtabmap-ros 0.23.7 installed, APT_RC=0 (see trial1.log for apt tail in local rtab_install_out.txt).
## First-run (60-75s background, use_sim_time true, namespace robot1, DetectionRate 2.0)
- Launch: rtabmap_go2_rgbd_odom.launch.py (rgbd_sync + rtabmap_slam, --delete_db_on_start, db=rtabmap_trial1.db)
- Log: trial1.log shows rgbd_sync approx_sync true, rtabmap subscribed, iterations Rate=0.50s RTAB-Map~0.08-0.11s Maps update~0.01s, 40+ iterations, no crash.
- Topics (actual names under /robot1/, NOT /robot1/rtabmap/):
  /robot1/mapData (MapData) hz~0.56Hz confirmed
  /robot1/cloud_map (PointCloud2) publisher confirmed via node info (hz low/latched, needs longer window)
  /robot1/grid_prob_map (OccupancyGrid) + /robot1/map (OccupancyGrid) publishers confirmed
  /robot1/info, /robot1/mapGraph, /robot1/rgbd_d435/rgbd_image, /robot1/octomap_* also present
  Nodes: /robot1/rgbd_sync, /robot1/rtabmap
- Artifacts: rtabmap_trial1.db ~48-49MB, trial1.log ~18KB
## Blockers / next
- No hard blocker. Soft issues: (1) missing d435 camera_info -> use relay; (2) color stream only 2-3Hz -> prefer d435 pair + relay, or降分辨率 if CPU bound; (3) map->odom owned by amcl so rtabmap publish_tf=false for now; for full SLAM (loop closure + own map->odom) need to disable amcl or set rtabmap publish_tf=true in isolated test; (4) /robot1/info hz not observed in short window, check QoS/latch.
- If CPU/MEM tight or need driving map: accept octomap route in parallel, or switch rtabmap to subscribe_rgbd with relayed rgbd_image.
- Did NOT touch slam/nav2/dog control, did NOT delete existing maps, did NOT publish cmd_vel.
