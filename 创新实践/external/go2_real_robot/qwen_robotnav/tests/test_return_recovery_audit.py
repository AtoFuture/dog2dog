"""A reordered release message must not hide a genuine interior motion command."""
import json
from pathlib import Path
import subprocess
import sys
import pytest
pytest.importorskip('rclpy')
import rosbag2_py
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from action_msgs.msg import GoalStatusArray,GoalStatus

SCRIPT=Path(__file__).resolve().parents[1]/'scripts/evaluate_return_recovery.py'


@pytest.mark.parametrize('bad_interior',[False,True])
def test_release_reordering_and_real_motion_are_distinguished(tmp_path,bad_interior):
    goal='01'*16
    report=dict(recorder_exit_code=0,navigation_return_success=True,truth_home_error_m=.2,
        truth_in_navigation_tf=False,static_map_loaded=False,return_hold_s=18.)
    (tmp_path/'report.json').write_text(json.dumps(report))
    (tmp_path/'mission_report.json').write_text(json.dumps(dict(return_attempt_id='test')))
    (tmp_path/'nav2.log').write_text('Stable map progress checker enabled\n[WARN] [110.000] Stable map progress stalled; requesting FollowPath recovery\n')
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(tmp_path/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    topics={
        '/clock':'rosgraph_msgs/msg/Clock','/frontier_trial/fault_events':'std_msgs/msg/String',
        '/robot1/cmd_vel':'geometry_msgs/msg/Twist','/g3/return_event':'std_msgs/msg/String',
        '/mission/recon_status':'std_msgs/msg/String','/robot1/navigate_to_pose/_action/status':'action_msgs/msg/GoalStatusArray'}
    for name,kind in topics.items():writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=kind,serialization_format='cdr'))
    def write(topic,msg,time_s):writer.write(topic,serialize_message(msg),int(time_s*1e9))
    def string(topic,value,t):write(topic,String(data=json.dumps(value)),t)
    string('/g3/return_event',dict(attempt_id='test',status='accepted',goal_id=goal),99.9)
    write('/clock',Clock(),99.999)
    before=Twist();before.linear.x=.02
    write('/robot1/cmd_vel',before,99.9995)
    string('/frontier_trial/fault_events',dict(status='return_hold_started',stamp_ns=0,wall_time_s=100.,duration_sim_s=18.,goal_id=goal),100.0001)
    for tick in range(1,180):
        clock=Clock();clock.clock.sec=tick//10;clock.clock.nanosec=(tick%10)*100_000_000
        write('/clock',clock,100.+tick*.1)
        command=Twist()
        if bad_interior and tick==50:command.linear.x=.02
        write('/robot1/cmd_vel',command,100.+tick*.1+.0001)
    clock=Clock();clock.clock.sec=18;write('/clock',clock,118.)
    command=Twist();command.angular.z=.4
    write('/robot1/cmd_vel',command,118.0001)
    string('/frontier_trial/fault_events',dict(status='return_hold_released',stamp_ns=18_000_000_000,wall_time_s=118.,duration_sim_s=18.),118.0002)
    status=GoalStatus();status.status=4;status.goal_info.goal_id.uuid=[1]*16
    statuses=GoalStatusArray();statuses.status_list=[status]
    write('/robot1/navigate_to_pose/_action/status',statuses,120.)
    string('/g3/return_event',dict(attempt_id='test',status='succeeded',goal_id=goal),120.001)
    string('/mission/recon_status',dict(return_attempt_id='test',state='returned'),120.002)
    del writer
    result=subprocess.run([sys.executable,str(SCRIPT),str(tmp_path)],capture_output=True,text=True,timeout=30)
    audit=json.loads((tmp_path/'return_recovery_validation.json').read_text())
    assert audit['nonzero_commands_in_receipt_order']==1+int(bad_interior)
    assert audit['nonzero_commands_during_hold']==int(bad_interior)
    assert audit['return_success'] and audit['stall_warnings_during_hold']==1
    assert audit['hold_injection_success']==(not bad_interior)
    assert result.returncode==(1 if bad_interior else 0)
