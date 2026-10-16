"""Closed ROS-message bags: reported success must not replace sensor/action proof."""
import importlib.util
import json
from pathlib import Path
import pytest
pytest.importorskip('rclpy')
import rosbag2_py
from rclpy.serialization import serialize_message
from action_msgs.msg import GoalStatus,GoalStatusArray
from geometry_msgs.msg import TransformStamped,Twist
from nav_msgs.msg import OccupancyGrid,Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

script=Path(__file__).resolve().parents[1]/'scripts/evaluate_dynamic_obstacle.py'
spec=importlib.util.spec_from_file_location('dynamic_audit',script)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


@pytest.mark.parametrize('missing',['none','lidar','action','emergency','truth','outbound','return'])
def test_reported_success_requires_independent_closed_bag_evidence(tmp_path,missing):
    (tmp_path/'report.json').write_text(json.dumps(dict(success=True,recorder_exit_code=0,
        emergency_guard_interventions=int(missing=='emergency'),truth_home_error_m=.2,
        truth_start=[0.,0.,.23,0.],truth_in_navigation_tf=missing=='truth')))
    (tmp_path/'dynamic_scene.json').write_text(json.dumps(dict(scenario='blocked',size_m=[.5,1.6,.8])))
    (tmp_path/'cart_motion.json').write_text(json.dumps(dict(success=True,start_stamp_ns=0,
        end_stamp_ns=15_000_000_000,anchor_world=[0,0,0])))
    (tmp_path/'dynamic_goals.json').write_text(json.dumps([
        dict(goal_id='01'*16,status=4,end_stamp_ns=20_000_000_000),
        dict(goal_id='02'*16,status=4,end_stamp_ns=25_000_000_000)]))
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(tmp_path/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    topics={'/clock':'rosgraph_msgs/msg/Clock','/frontier_trial/cart_events':'std_msgs/msg/String',
        '/frontier_trial/gt_pose':'tf2_msgs/msg/TFMessage','/robot1/tf_static':'tf2_msgs/msg/TFMessage',
        '/robot1/cmd_vel':'geometry_msgs/msg/Twist','/deepseek/nav_cmd':'geometry_msgs/msg/Twist',
        '/robot1/velodyne':'sensor_msgs/msg/LaserScan','/robot1/odometry/filtered':'nav_msgs/msg/Odometry',
        '/robot1/local_costmap/costmap':'nav_msgs/msg/OccupancyGrid',
        '/robot1/navigate_to_pose/_action/status':'action_msgs/msg/GoalStatusArray'}
    for name,kind in topics.items():writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=kind,serialization_format='cdr'))
    serial=0
    def write(topic,msg,t):
        nonlocal serial
        serial+=1;writer.write(topic,serialize_message(msg),int((100+t)*1e9)+serial)
    sensor=TransformStamped();sensor.header.frame_id='base_link';sensor.child_frame_id='velodyne'
    sensor.transform.translation.x=.22;sensor.transform.rotation.w=1.
    write('/robot1/tf_static',TFMessage(transforms=[sensor]),0)
    for t,x in [(0,0.),(2,0.),(17,0.),(20,2.2),(25,.2)]:
        if missing=='outbound' and t==20:x=5.
        if missing=='return' and t==25:x=1.
        clock=Clock();clock.clock.sec=t;write('/clock',clock,t)
        cart=dict(x=1.5,y=0. if t<15 else 2.4,yaw=0.)
        write('/frontier_trial/cart_events',String(data=json.dumps(cart)),t)
        tr=TransformStamped();tr.child_frame_id='robot1_my_bot';tr.transform.translation.x=x
        tr.transform.translation.z=.23;tr.transform.rotation.w=1.
        write('/frontier_trial/gt_pose',TFMessage(transforms=[tr]),t)
        odom=Odometry();odom.pose.pose.position.x=x;odom.pose.pose.orientation.w=1.
        write('/robot1/odometry/filtered',odom,t)
        scan=LaserScan(angle_min=-.01,angle_increment=.01,range_min=.1,range_max=10.,ranges=[1.03]*3)
        if missing!='lidar' and t==2:write('/robot1/velodyne',scan,t)
        grid=OccupancyGrid();grid.header.frame_id='odom';grid.info.width=grid.info.height=80
        grid.info.resolution=.05;grid.info.origin.position.x=grid.info.origin.position.y=-1.
        grid.data=[0]*6400
        if t<15:grid.data[20*80+45]=100
        write('/robot1/local_costmap/costmap',grid,t)
        velocity=Twist();velocity.linear.x=.03 if t==17 else 0.
        write('/robot1/cmd_vel',velocity,t);write('/deepseek/nav_cmd',velocity,t)
    statuses=[]
    for index in (1,2):
        status=GoalStatus();status.status=4;status.goal_info.goal_id.uuid=[index if missing!='action' else 9]*16
        statuses.append(status)
    write('/robot1/navigate_to_pose/_action/status',GoalStatusArray(status_list=statuses),25)
    del writer
    result=module.evaluate(tmp_path)
    assert result['success']==(missing=='none')
    assert (tmp_path/'dynamic_trajectory.png').exists()
