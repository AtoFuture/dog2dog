"""Deterministic Nav2 benchmark, using the existing single safety publisher.

This isolates local navigation from frontier/LLM and person-count variability.
Goals use the estimated map pose. Simulator truth is used only in evaluation.
"""
import json
import math
from pathlib import Path
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def run_trial(monitor, root, home, baseline, start, spin_until):
    client=ActionClient(monitor,NavigateToPose,'/robot1/navigate_to_pose')
    goals=[];driver=start(['python3',str(Path(__file__).with_name('drive_dynamic_cart.py')),str(root)],'cart_motion.log')
    monitor.running=True
    def wait(condition, timeout):
        def checked():
            if driver.poll() is not None and driver.returncode!=0:
                raise RuntimeError('Dynamic cart driver failed; inspect cart_motion.json')
            return condition()
        if not spin_until(checked,timeout):raise RuntimeError('Dynamic navigation deadline expired')
    try:
        if not client.wait_for_server(timeout_sec=5):raise RuntimeError('Nav2 action unavailable')
        for phase,x,y in [('outbound',home[0]+2.4*math.cos(home[2]),home[1]+2.4*math.sin(home[2])),
                          ('return',home[0],home[1])]:
            goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
            goal.pose.header.stamp=monitor.get_clock().now().to_msg()
            goal.pose.pose.position.x=x;goal.pose.pose.position.y=y
            goal.pose.pose.orientation.w=1.
            request=client.send_goal_async(goal)
            wait(request.done,10);handle=request.result()
            if not handle.accepted:raise RuntimeError('Dynamic trial goal rejected')
            event=dict(phase=phase,goal_id=bytes(handle.goal_id.uuid).hex(),target_map=[x,y],
                       start_stamp_ns=monitor.get_clock().now().nanoseconds)
            goals.append(event)
            result=handle.get_result_async()
            try:wait(result.done,180)
            except BaseException:
                handle.cancel_goal_async();raise
            event.update(status=result.result().status,end_stamp_ns=monitor.get_clock().now().nanoseconds)
            if event['status']!=GoalStatus.STATUS_SUCCEEDED:raise RuntimeError('Dynamic trial '+phase+' goal failed')
            if phase=='outbound':
                wait(lambda:driver.poll() is not None,100)
        monitor.running=False;spin_until(lambda:False,2)
        end=monitor.poses[-1];error=math.hypot(end[1]-baseline[1],end[2]-baseline[2])
        return dict(success=error<=.4,dynamic_navigation_success=error<=.4,
                    truth_home_error_m=error,truth_start=baseline[1:],truth_end=end[1:],home_map=home)
    finally:
        monitor.running=False;client.destroy()
        (root/'dynamic_goals.json').write_text(json.dumps(goals,indent=2))
