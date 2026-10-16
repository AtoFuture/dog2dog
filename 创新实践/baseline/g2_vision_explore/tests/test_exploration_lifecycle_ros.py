"""Real DDS and action protocol, synthetic static maps and pose, no actuator publisher."""
import json
from pathlib import Path
import sys
import threading
import time
import numpy as np
import pytest
rclpy = pytest.importorskip('rclpy')
sys.path.insert(0, str(Path(__file__).parents[2]/'g3_integration_eval'))
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from vision_explorer.explorer_node import VisionExplorer, _MAP_QOS
from g3_integration_eval.state_machine_node import G3StateMachineNode
from g3_integration_eval.state_machine import State


@pytest.mark.parametrize('reason', ['no_frontiers', 'time_budget', 'navigation_failures'])
def test_autonomous_end_to_return_without_observer_return_command(reason):
    rclpy.init(args=['--ros-args',
        '-p','vision_explorer:task_completion_enabled:=true',
        '-p','vision_explorer:require_mission_state:=true',
        '-p','vision_explorer:tick_period_s:=0.05',
        '-p','vision_explorer:no_frontier_confirm_s:=0.3',
        '-p','vision_explorer:exploration_budget_s:=1.5',
        '-p','vision_explorer:max_consecutive_failures:=2',
        '-p','vision_explorer:nav_action:=/robot1/navigate_to_pose',
        '-p','g3_state_machine:enable_exploration_completion:=true',
        '-p','g3_state_machine:require_navigation_handoff:=true',
        '-p','g3_state_machine:enable_navigation_transmission:=true'])
    explorer=VisionExplorer();g3=G3StateMachineNode();peer=Node('static_mission_fixture')
    # Only localization is supplied directly; selection uses actual occupancy maps.
    explorer._robot_xy=lambda:(0.,0.)
    counts={'active':0,'max_active':0,'home':0,'canceled':0,'explore':0,'failed':0}
    done=threading.Event()
    def execute(handle):
        target=handle.request.pose.pose.position
        home=abs(target.x)+abs(target.y)<1e-6
        counts['active']+=1;counts['max_active']=max(counts['max_active'],counts['active'])
        try:
            if home:
                counts['home']+=1
                handle.succeed()
            else:
                counts['explore']+=1
                if reason=='navigation_failures':
                    counts['failed']+=1;handle.abort()
                    return NavigateToPose.Result()
                deadline=time.monotonic()+8
                while not handle.is_cancel_requested and not done.is_set() and time.monotonic()<deadline:
                    time.sleep(.01)
                if handle.is_cancel_requested:
                    counts['canceled']+=1;handle.canceled()
                else:handle.abort()
            return NavigateToPose.Result()
        finally:counts['active']-=1
    server=ActionServer(peer,NavigateToPose,'/robot1/navigate_to_pose',execute,
        cancel_callback=lambda goal:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
    publisher=peer.create_publisher(OccupancyGrid,'/map',_MAP_QOS)
    results=[]
    peer.create_subscription(String,'/g3/exploration_result',lambda msg:results.append(json.loads(msg.data)),_MAP_QOS)
    data=np.zeros((40,40),dtype=np.int8)
    if reason=='no_frontiers':
        data[0,:]=data[-1,:]=data[:,0]=data[:,-1]=100
    def publish_map():
        msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp=peer.get_clock().now().to_msg()
        msg.info.width=msg.info.height=40;msg.info.resolution=.1
        msg.info.origin.position.x=msg.info.origin.position.y=-2.;msg.info.origin.orientation.w=1.
        msg.data=data.ravel().tolist();publisher.publish(msg)
    peer.create_timer(.05,publish_map)
    executor=MultiThreadedExecutor(num_threads=4)
    for node in (explorer,g3,peer):executor.add_node(node)
    thread=threading.Thread(target=executor.spin);thread.start()
    def wait(predicate, timeout=10):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if predicate():return
            time.sleep(.02)
        raise AssertionError('DDS mission deadline: '+str(counts))
    try:
        wait(lambda:explorer._grid is not None and explorer._nav.server_is_ready())
        g3._on_command(String(data='explore'))  # Only operator command in the mission.
        wait(lambda:bool(results) and g3._state is State.STANDBY and counts['home']==1)
        assert results[-1]['reason']==reason
        assert results[-1]['completed'] is (reason=='no_frontiers')
        assert counts['max_active']==1 and counts['active']==0
        assert counts['explore']==({'no_frontiers':0,'time_budget':1,'navigation_failures':2}[reason])
        assert counts['canceled']==(1 if reason=='time_budget' else 0)
        assert counts['failed']==(2 if reason=='navigation_failures' else 0)
        assert explorer._attempt is None and not explorer._sm.should_explore
        assert not peer.get_publishers_info_by_topic('/robot1/cmd_vel')
    finally:
        done.set();explorer.stop_navigation();executor.shutdown(timeout_sec=5);thread.join(timeout=5)
        server.destroy()
        for node in (explorer,g3,peer):node.destroy_node()
        rclpy.shutdown()
