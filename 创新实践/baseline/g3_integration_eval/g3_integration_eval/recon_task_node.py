"""Find declared targets, release exploration, return through G3 and save a report."""
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from g3_integration_eval.recon_task import confirmed_targets


class ReconTaskNode(Node):
    def __init__(self):
        super().__init__('recon_task')
        self.declare_parameter('required_classes',['person'])
        self.declare_parameter('minimum_exploration_goals',1)
        self.declare_parameter('recon_output_dir','')
        self.required = list(self.get_parameter('required_classes').value)
        self.minimum_goals = int(self.get_parameter('minimum_exploration_goals').value)
        directory = self.get_parameter('recon_output_dir').value
        if not directory or not self.required or self.minimum_goals < 0:
            raise ValueError('Set recon_output_dir, required classes and non-negative exploration goal count')
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True)
        if (self.directory/'mission_report.json').exists():
            raise ValueError('Use a fresh reconnaissance output directory')
        self.state='waiting';self.mission=None;self.started_stamp=None
        self.targets={};self.completed_goals=set();self.return_attempt=None;self.return_requested=False
        self.history=[]
        self.publisher=self.create_publisher(String,'/mission/recon_status',10)
        self.command=self.create_publisher(String,'/g3/command',10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/mission_state',self._mission,qos)
        self.create_subscription(String,'/mission/detection_evidence',self._evidence,10)
        self.create_subscription(String,'/exploration/events',self._navigation,30)
        self.create_subscription(String,'/g3/return_event',self._return_event,30)
        self._save()

    def _save(self):
        report=dict(state=self.state,objective='find declared classes with two tracked JPEG observations, then return',
            required_classes=self.required,minimum_exploration_goals=self.minimum_goals,
            completed_exploration_goals=len(self.completed_goals),targets=self.targets,
            return_requested=self.return_requested,return_attempt_id=self.return_attempt,
            success=self.state=='returned',history=self.history)
        temp=self.directory/'mission_report.json.tmp'
        temp.write_text(json.dumps(report,ensure_ascii=False,indent=2));temp.replace(self.directory/'mission_report.json')
        self.publisher.publish(String(data=json.dumps(report,ensure_ascii=False)))

    def _transition(self,state):
        if self.state==state:return
        self.state=state;self.history.append(dict(state=state,monotonic_time=time.monotonic()))
        self._save()

    def _mission(self,msg):
        self.mission=msg.data
        if self.mission=='exploring' and self.started_stamp is None:
            self.started_stamp=self.get_clock().now().nanoseconds
            self._transition('searching')
        elif self.mission=='estop':self._transition('failed')
        elif self.mission=='returning' and not self.return_requested:self._transition('interrupted')

    def _evidence(self,msg):
        if self.started_stamp is None or self.return_requested or self.state!='searching':return
        try:
            self.targets=confirmed_targets(json.loads(msg.data),self.required,min_stamp_ns=self.started_stamp)
        except (ValueError,TypeError,AttributeError):return
        self._maybe_return()

    def _navigation(self,msg):
        if self.state!='searching':return
        try:
            event=json.loads(msg.data)
            if event['status']=='succeeded' and event.get('geodesic_dist_m',0)>=.75:
                self.completed_goals.add(event['attempt_id'])
        except (KeyError,TypeError,ValueError):return
        self._maybe_return()

    def _maybe_return(self):
        if (not self.return_requested and self.mission=='exploring'
            and set(self.required)<=set(self.targets) and len(self.completed_goals)>=self.minimum_goals):
            self.return_requested=True
            self._transition('return_requested')
            self.command.publish(String(data='RETURN'))
        else:self._save()

    def _return_event(self,msg):
        if not self.return_requested or self.state in ('returned','failed'):return
        try:
            event=json.loads(msg.data);attempt=event['attempt_id'];status=event['status']
        except (ValueError,KeyError,TypeError):return
        if self.return_attempt is None and status in ('waiting_for_server','goal_request_sent','accepted'):
            self.return_attempt=attempt
        if not attempt or attempt!=self.return_attempt:return
        if status=='succeeded':self._transition('returned')
        elif status in ('aborted','canceled','rejected','server_unavailable'):self._transition('failed')


    def destroy_node(self):
        if self.state not in ('returned','failed','interrupted','aborted'):
            self._transition('aborted')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=ReconTaskNode();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
