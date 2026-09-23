# AWS Small Warehouse · Gazebo Sim 6 compatibility layer

This directory is an additive compatibility layer. It does not modify the
downloaded AWS repository or anything under `baseline/`.

Gazebo Sim 6 rejects the original static roof and ground models because their
legacy inertia tensors do not satisfy its stricter validation. Static models do
not use inertia in simulation, so this layer supplies copies of only those two
models with the unused inertial blocks removed. All meshes and every other
model are loaded from the original AWS repository.

The compatibility world also adds Gazebo Sim's Physics, UserCommands,
SceneBroadcaster, and Sensors systems. The original Classic world does not load
these systems, so a spawned Go2 would have no live lidar or camera data without
this additive world file.

Run these commands from two host terminals. The scripts enter the existing
ROS / Gazebo GUI container themselves and use the isolated Gazebo partition
`aws_small_warehouse_20260921` and ROS domain `142`.

Terminal 1 -- start the warehouse:

```bash
/home/wy/data1/创新实践/external/aws-small-warehouse-gzsim-compat/run_gzsim.sh
```

After the warehouse has loaded, use terminal 2 to unpause the world and spawn
Go2 without opening another RViz window:

```bash
/home/wy/data1/创新实践/external/aws-small-warehouse-gzsim-compat/spawn_go2.sh
```

The launch process must remain running while the robot is being simulated.
The Go2 entity name is `robot1_my_bot`, and its ROS topics use the `/robot1`
namespace. To run another isolated pair, set the same custom partition and ROS
domain for both commands:

```bash
IGN_PARTITION=aws_small_warehouse_2 ROS_DOMAIN_ID=143 \
  /home/wy/data1/创新实践/external/aws-small-warehouse-gzsim-compat/run_gzsim.sh

IGN_PARTITION=aws_small_warehouse_2 ROS_DOMAIN_ID=143 \
  /home/wy/data1/创新实践/external/aws-small-warehouse-gzsim-compat/spawn_go2.sh
```
