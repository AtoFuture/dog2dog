"""Find declared targets, release exploration, return through G3 and save a report."""
import json
from pathlib import Path
import time
import math
from rclpy.clock import Clock, ClockType
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from g3_integration_eval.recon_task import confirmed_instances
from g3_integration_eval.moving_identity import MovingIdentityResolver


class ReconTaskNode(Node):
    def __init__(self):
        super().__init__('recon_task')
        self.declare_parameter('required_classes',['person'])
        self.declare_parameter('recon_identity_policy','motion_appearance_v1')
        self.identity_policy=self.get_parameter('recon_identity_policy').value
        if self.identity_policy not in ('stationary','motion_appearance_v1'):
            raise ValueError('Unknown recon identity policy')
        self.identity_resolver=MovingIdentityResolver()
        self.declare_parameter('minimum_exploration_goals',1)
        self.declare_parameter('minimum_observation_duration_s',0.)
        self.declare_parameter('minimum_observation_viewpoints',0)
        self.minimum_observation_viewpoints=int(self.get_parameter('minimum_observation_viewpoints').value)
        if self.minimum_observation_viewpoints<0:raise ValueError('Observation viewpoint count must be non-negative')
        self.completed_views=set()
        self.declare_parameter('required_target_recency_s',0.)
        self.required_target_recency_s=float(self.get_parameter('required_target_recency_s').value)
        if not math.isfinite(self.required_target_recency_s) or self.required_target_recency_s<0:
            raise ValueError('Target recency must be finite and non-negative')
        self.minimum_observation_duration_s=float(self.get_parameter('minimum_observation_duration_s').value)
        if not math.isfinite(self.minimum_observation_duration_s) or self.minimum_observation_duration_s<0:
            raise ValueError('Minimum observation duration must be finite and non-negative')
        self.declare_parameter('required_person_count',1)
        self.declare_parameter('recon_output_dir','')
        self.declare_parameter('search_timeout_s',600.)
        self.declare_parameter('no_candidate_timeout_s',30.)
        self.required = list(self.get_parameter('required_classes').value)
        self.required_person_count=int(self.get_parameter('required_person_count').value)
        self.minimum_goals = int(self.get_parameter('minimum_exploration_goals').value)
        directory = self.get_parameter('recon_output_dir').value
        if not directory or not self.required or self.minimum_goals < 0 or self.required_person_count < 1:
            raise ValueError('Set recon_output_dir, required classes and non-negative exploration goal count')
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True)
        if (self.directory/'mission_report.json').exists():
            raise ValueError('Use a fresh reconnaissance output directory')
        self.state='waiting';self.mission=None;self.started_stamp=None
        self.targets={};self.completed_goals=set();self.return_attempt=None;self.return_requested=False
        self.search_timeout=float(self.get_parameter('search_timeout_s').value)
        self.no_candidate_timeout=float(self.get_parameter('no_candidate_timeout_s').value)
        if not all(math.isfinite(v) and v>0 for v in (self.search_timeout,self.no_candidate_timeout)):
            raise ValueError('Search and no-candidate budgets must be finite and positive')
        self.search_started_at=None;self.unavailable_since=None;self.return_reason=None
        self.search_budget_paused=False
        self.history=[]
        self.publisher=self.create_publisher(String,'/mission/recon_status',10)
        self.command=self.create_publisher(String,'/g3/command',10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/mission_state',self._mission,qos)
        self.create_subscription(String,'/mission/detection_evidence',self._evidence,10)
        self.create_subscription(String,'/exploration/events',self._navigation,30)
        self.create_subscription(String,'/g3/return_event',self._return_event,30)
        self._budget_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.,self._check_budget,clock=self._budget_clock)
        self._save()

    def _save(self):
        report=dict(state=self.state,objective='confirm declared counts of distinct targets with two JPEG observations per identity, then return',
            required_classes=self.required,required_counts={c:self.required_person_count if c=='person' else 1 for c in self.required},
            identity_policy=self.identity_policy,identity_diagnostics=self.identity_resolver.diagnostics,minimum_exploration_goals=self.minimum_goals,
            completed_exploration_goals=len(self.completed_goals),targets=self.targets,
            minimum_observation_duration_s=self.minimum_observation_duration_s,
            minimum_observation_viewpoints=self.minimum_observation_viewpoints,completed_observation_viewpoints=len(self.completed_views),
            required_target_recency_s=self.required_target_recency_s,targets_fresh=self._targets_recent(),
            observation_window_pending=(not self._observation_duration_complete() and self._targets_recent() and self.objective_confirmed() and len(self.completed_goals)>=self.minimum_goals and self._views_complete()),
            return_requested=self.return_requested,return_attempt_id=self.return_attempt,
            success=self.state=='returned',return_success=self.state in ('returned','returned_incomplete'),
            return_reason=self.return_reason,missing_counts=self.missing_counts(),
            search_timeout_s=self.search_timeout,history=self.history)
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
            self.search_started_at=time.monotonic()
            self._transition('searching')
        elif self.mission=='estop':self._transition('failed')
        elif self.mission=='returning' and not self.return_requested:self._transition('interrupted')

    def _evidence(self,msg):
        if self.started_stamp is None or self.return_requested or self.state!='searching':return
        try:
            payload=json.loads(msg.data)
            if self.identity_policy=='motion_appearance_v1':
                self.targets=self.identity_resolver.resolve(payload,self.required,min_stamp_ns=self.started_stamp)
            else:
                self.targets=confirmed_instances(payload,self.required,min_stamp_ns=self.started_stamp)
        except (ValueError,TypeError,AttributeError):return
        self._maybe_return()

    def _navigation(self,msg):
        if self.state!='searching':return
        try:
            event=json.loads(msg.data)
            if event['status']=='search_unavailable' and event.get('reason') in ('NO_FRONTIER','NO_CANDIDATE','SMALL_FRONTIERS'):
                if getattr(self,'unavailable_since',None) is None:self.unavailable_since=time.monotonic()
            elif event['status'] in ('sending','brain_requested','search_unavailable'):self.unavailable_since=None
            if event['status']=='succeeded' and event.get('purpose')=='observe_targets' and event.get('geodesic_dist_m',0)>=.5:
                self.completed_views.add(event['attempt_id'])
            if event['status']=='succeeded' and event.get('geodesic_dist_m',0)>=.75:
                self.completed_goals.add(event['attempt_id'])
        except (KeyError,TypeError,ValueError):return
        self._maybe_return()

    def objective_confirmed(self):
        return all(sum(t.get('class_name',key.split(':')[0])==c for key,t in self.targets.items())
                   >= (getattr(self,'required_person_count',1) if c=='person' else 1) for c in self.required)

    def missing_counts(self):
        return {c:max(0,(getattr(self,'required_person_count',1) if c=='person' else 1)
                      -sum(t.get('class_name',key.split(':')[0])==c for key,t in self.targets.items())) for c in self.required}

    def request_return(self,reason):
        if self.return_requested or self.state!='searching' or self.mission!='exploring':return
        self.return_requested=True;self.return_reason=reason
        self._transition('return_requested')
        self.command.publish(String(data='RETURN'))

    def _check_budget(self):
        if self.state!='searching' or self.mission!='exploring':return
        self._maybe_return()
        if self.return_requested or getattr(self,'search_budget_paused',False):return
        now=time.monotonic()
        if self.search_started_at is not None and now-self.search_started_at>=self.search_timeout:
            self.request_return('search_budget_exhausted')
        elif self.unavailable_since is not None and now-self.unavailable_since>=self.no_candidate_timeout:
            self.request_return('no_reachable_viewpoints')

    def _views_complete(self):
        return len(getattr(self,'completed_views',()))>=getattr(self,'minimum_observation_viewpoints',0)

    def _targets_recent(self):
        recency=getattr(self,'required_target_recency_s',0.)
        if recency==0:return True
        now=self.get_clock().now().nanoseconds
        return all(sum(t.get('class_name',key.split(':')[0])==c and
                       isinstance(t.get('last_seen_stamp_ns'),int) and
                       0<=now-t['last_seen_stamp_ns']<=recency*1e9
                       for key,t in self.targets.items()) >=
                   (getattr(self,'required_person_count',1) if c=='person' else 1) for c in self.required)

    def _observation_duration_complete(self):
        minimum=getattr(self,'minimum_observation_duration_s',0.)
        return minimum==0 or (self.started_stamp is not None and
            (self.get_clock().now().nanoseconds-self.started_stamp)*1e-9>=minimum)

    def _maybe_return(self):
        observed=self._observation_duration_complete()
        if (not self.return_requested and self.mission=='exploring'
            and observed and self._views_complete() and self._targets_recent() and self.objective_confirmed() and len(self.completed_goals)>=self.minimum_goals):
            self.request_return('objective_completed')
        else:self._save()

    def _return_event(self,msg):
        if not self.return_requested or self.state in ('returned','returned_incomplete','failed'):return
        try:
            event=json.loads(msg.data);attempt=event['attempt_id'];status=event['status']
        except (ValueError,KeyError,TypeError):return
        if self.return_attempt is None and status in ('waiting_for_server','goal_request_sent','accepted'):
            self.return_attempt=attempt
        if not attempt or attempt!=self.return_attempt:return
        if status=='succeeded':
            self._transition('returned' if getattr(self,'return_reason',None) in (None,'objective_completed') else 'returned_incomplete')
        elif status in ('aborted','canceled','rejected','server_unavailable'):self._transition('failed')


    def destroy_node(self):
        if self.state not in ('returned','returned_incomplete','failed','interrupted','aborted'):
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
