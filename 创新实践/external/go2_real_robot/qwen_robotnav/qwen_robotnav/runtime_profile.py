"""One validated configuration for the Gazebo navigation runtime."""
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
from urllib.parse import urlparse
import yaml

KEYS={
 'topics':set('nav_command guarded_input actuator_command scan odometry imu battery mission command guard_status map planning_map action tf tf_static'.split()),
 'frames':{'map','odom','base'},
 'guard':set('clearance_margin_m prediction_horizon_s prediction_speed_mps rate_hz command_timeout_s sensor_timeout_s battery_timeout_s mission_timeout_s stamp_future_tolerance_s linear_max angular_max front_half_angle_rad front_clearance_m max_tilt_rad clearance_recovery_enabled'.split()),
 'scaler':set('linear_scale linear_max linear_min nav_deadband angular_scale angular_max linear_accel linear_decel angular_accel angular_decel ramp_rate cmd_timeout'.split()),
 'navigation':{'robot_radius_m','inflation_radius_m','desired_linear_velocity_mps','goal_turn_clearance_m','return_turn_handoff_enabled','reverse_retreat_enabled','reverse_retreat_max_extent_m'},
 'mission':set('heartbeat_period_s recovery_stale_s recovery_stable_s recovery_timeout_s low_battery_threshold home_x home_y home_yaw'.split()),
 'explorer':set('max_bootstrap_scans require_path_clearance tick_period_s min_goal_distance_m task_completion_enabled exploration_budget_s coverage_stall_s coverage_min_gain_m2 candidate_wait_s no_frontier_confirm_s max_region_failures max_consecutive_failures'.split()),
 'decision':set('candidate_selector mission_instruction brain_base_url brain_model brain_timeout brain_max_candidates'.split()),
 'battery_model':{'initial_percentage','moving_drain_per_sec','idle_drain_per_sec','publish_rate_hz'},
}
CORRIDOR_KEYS=set(
    'enabled steering_enabled max_width_m min_span_m wall_tolerance_m '
    'lookahead_m heading_gain turn_rate_radps max_heading_error_rad '
    'approach_distance_m path_lookahead_m alignment_tolerance_rad '
    'alignment_confirmations turn_settle_enabled reversal_turn_enabled turn_recenter_enabled'.split())
OPTIONAL_CORRIDOR_KEYS=set(
    'steering_enabled approach_distance_m path_lookahead_m '
    'alignment_tolerance_rad alignment_confirmations turn_settle_enabled reversal_turn_enabled turn_recenter_enabled'.split())
RETURN_LOCALIZATION_KEYS=set(
    'enabled stale_s jump_translation_m jump_yaw_rad stationary_linear_mps '
    'stationary_angular_radps stationary_drift_m stationary_drift_yaw_rad '
    'position_tolerance_m yaw_tolerance_rad stable_s verification_timeout_s '
    'max_corrections scan_match_required scan_match_rmse_m '
    'scan_match_min_inlier_ratio scan_match_max_correspondence_m '
    'scan_match_min_points scan_match_max_points scan_match_max_iterations '
    'scan_match_convergence_m max_scan_correction_m '
    'max_scan_correction_yaw_rad'.split())


def load_profile(path):
    return validate_profile(yaml.safe_load(Path(path).read_text()))


def validate_profile(value):
    sections=dict(KEYS)
    if isinstance(value,dict) and 'corridor' in value:
        sections['corridor']=CORRIDOR_KEYS
    if isinstance(value,dict) and 'return_localization' in value:
        sections['return_localization']=RETURN_LOCALIZATION_KEYS
    if not isinstance(value,dict) or set(value)!={'schema_version','backend','use_sim_time',*sections}:
        raise ValueError('Unknown or missing runtime profile fields')
    if type(value['schema_version']) is not int or value['schema_version']!=1 or value['backend']!='gazebo_go2' or value['use_sim_time'] is not True:
        raise ValueError('Only the measured gazebo_go2 simulation profile is supported')
    for section,keys in sections.items():
        data=value[section]
        required = keys-OPTIONAL_CORRIDOR_KEYS if section=='corridor' else keys
        if section=='navigation':required=keys-{'goal_turn_clearance_m','return_turn_handoff_enabled','reverse_retreat_enabled','reverse_retreat_max_extent_m'}
        if section=='guard':required=keys-{'clearance_recovery_enabled'}
        if not isinstance(data,dict) or not required <= set(data) <= keys:raise ValueError('Unknown or missing '+section+' fields')
        for key,item in data.items():
            if section=='navigation' and key in ('return_turn_handoff_enabled','reverse_retreat_enabled'):
                if type(item) is not bool:raise ValueError('Expected boolean navigation '+key)
                continue
            if section in ('corridor','return_localization','guard') and key in ('enabled','steering_enabled','scan_match_required','turn_settle_enabled','reversal_turn_enabled','turn_recenter_enabled','clearance_recovery_enabled'):
                if type(item) is not bool:raise ValueError('Expected boolean '+section+' '+key)
                continue
            if section=='return_localization' and key in ('max_corrections','scan_match_min_points','scan_match_max_points','scan_match_max_iterations'):
                if type(item) is not int or item < (0 if key=='max_corrections' else 1):raise ValueError('Invalid integer '+key)
                continue
            if section=='corridor' and key=='alignment_confirmations':
                if type(item) is not int or not 2<=item<=6:raise ValueError('Invalid integer '+key)
                continue
            if section=='decision':continue
            if section=='explorer' and key in ('task_completion_enabled','require_path_clearance'):
                if item is not True:raise ValueError('Formal runtime requires '+key)
                continue
            if section in ('topics','frames'):
                if not isinstance(item,str) or not item.strip() or any(c.isspace() for c in item):raise ValueError('Invalid '+key)
                if section=='topics' and not item.startswith('/'):raise ValueError('Use absolute topics')
            else:
                if isinstance(item,bool) or not isinstance(item,(int,float)) or not math.isfinite(item):raise ValueError('Invalid numeric '+key)
                if (not key.startswith('home_') and item<=0
                        and not (section=='navigation' and key=='goal_turn_clearance_m' and item==0)):
                    raise ValueError('Expected positive '+key)
    t,g,s,n,m=(value[k] for k in ('topics','guard','scaler','navigation','mission'))
    if len(set(t.values()))!=len(t):raise ValueError('Runtime topics must be distinct')
    if len(set(value['frames'].values()))!=3:raise ValueError('Runtime frames must be distinct')
    if not 1<=g['rate_hz']<=100 or g['linear_max']>.03 or g['angular_max']>.8:raise ValueError('Unvalidated simulator guard limits')
    if not 0<g['front_half_angle_rad']<=math.pi/2 or not 0<g['max_tilt_rad']<math.pi/2:raise ValueError('Invalid guard angle')
    if g['front_clearance_m']<.24 or g['sensor_timeout_s']>1 or g['command_timeout_s']>.5:raise ValueError('Weaker than measured simulator guard')
    if g['clearance_margin_m']<.05 or g['prediction_horizon_s']<.5 or g['prediction_speed_mps']<.4:
        raise ValueError('Weaker than measured footprint prediction envelope')
    if g['mission_timeout_s']<=2*m['heartbeat_period_s']:raise ValueError('Heartbeat margin is too small')
    if m['recovery_timeout_s']<=m['recovery_stable_s'] or not 0<m['low_battery_threshold']<1:raise ValueError('Invalid mission budgets')
    if s['linear_min']>s['linear_max'] or s['linear_max']>g['linear_max']:raise ValueError('Scaler exceeds guard limits')
    if not .30<=n['desired_linear_velocity_mps']<=.34 or not s['linear_min']<=n['desired_linear_velocity_mps']*s['linear_scale']<=s['linear_max']:
        raise ValueError('Uncalibrated simulator gait velocity range or scale')
    if n['inflation_radius_m']<n['robot_radius_m'] or n['robot_radius_m']<.4:raise ValueError('Invalid navigation clearance')
    if not .5<=n.get('reverse_retreat_max_extent_m',1.5)<=2.:raise ValueError('Unvalidated reverse odometry excursion limit')
    if n.get('goal_turn_clearance_m',0)>0 and n['goal_turn_clearance_m']<n['robot_radius_m']+g['clearance_margin_m']:
        raise ValueError('Turning disk must include the body and clearance margin')
    if not 0<value['battery_model']['initial_percentage']<=1:raise ValueError('Invalid simulated initial battery')
    if t['action']!='/robot1/navigate_to_pose':raise ValueError('This backend requires the robot1 Nav2 action')
    if 'corridor' in value:
        c=value['corridor']
        if not (1. <= c['max_width_m'] <= 2. and 1. <= c['min_span_m'] <= 3.
                and .02 <= c['wall_tolerance_m'] <= .08 and .25 <= c['lookahead_m'] <= 2.
                and .1 <= c['heading_gain'] <= 2. and .1 <= c['turn_rate_radps'] <= .4
                and .12 <= c['max_heading_error_rad'] <= .35
                and .5 <= c.get('approach_distance_m',1.) <= 1.5
                and .5 <= c.get('path_lookahead_m',1.) <= 2.
                and .04 <= c.get('alignment_tolerance_rad',.08) < c['max_heading_error_rad']):
            raise ValueError('Unvalidated corridor guidance limits')
    if 'return_localization' in value:
        r=value['return_localization']
        if (r['stable_s']>=r['verification_timeout_s']
                or not 0<r['scan_match_min_inlier_ratio']<=1
                or r['scan_match_min_points']>r['scan_match_max_points']
                or r['position_tolerance_m']>.4
                or r['max_scan_correction_m']>2.0
                or r['max_scan_correction_yaw_rad']>math.pi/2):
            raise ValueError('Unvalidated return localization limits')
    e=value['explorer']
    if type(e['max_bootstrap_scans']) is not int or not 1<=e['max_bootstrap_scans']<=3:raise ValueError('Expected 1..3 bootstrap scans')
    if e['candidate_wait_s']<=30:raise ValueError('Candidate wait must exceed blacklist TTL')
    for key in ('max_region_failures','max_consecutive_failures'):
        if type(e[key]) is not int:raise ValueError('Expected integer '+key)
    d=value['decision']
    if d['candidate_selector'] not in ('deepseek','deterministic'):raise ValueError('Invalid candidate selector')
    if not isinstance(d['mission_instruction'],str) or not d['mission_instruction'].strip() or len(d['mission_instruction'])>2000:raise ValueError('Invalid mission instruction')
    if not isinstance(d['brain_model'],str) or not d['brain_model'].strip():raise ValueError('Invalid brain model')
    if isinstance(d['brain_timeout'],bool) or not isinstance(d['brain_timeout'],(int,float)) or not math.isfinite(d['brain_timeout']) or not .1<=d['brain_timeout']<=60:raise ValueError('Invalid brain timeout')
    if type(d['brain_max_candidates']) is not int or not 1<=d['brain_max_candidates']<=16:raise ValueError('Invalid brain candidate limit')
    if not isinstance(d['brain_base_url'],str):raise ValueError('Invalid brain endpoint')
    url=urlparse(d['brain_base_url'])
    if url.scheme!='https' or not url.hostname or url.username or url.password or url.query or url.fragment:raise ValueError('Use credential-free HTTPS brain endpoint')
    return value


def profile_hash(profile):
    return hashlib.sha256(json.dumps(profile,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def ros_parameters(p, runtime_root=None):
    t,f,m=p['topics'],p['frames'],p['mission']
    common={'use_sim_time':p['use_sim_time']};d=p['decision']
    result={
      'g3_sim_battery':{'ros__parameters':dict(common,battery_topic=t['battery'],cmd_vel_topic=t['actuator_command'],**p['battery_model'])},
      'cmd_vel_scaler':{'ros__parameters':dict(common,input_topic=t['nav_command'],output_topic=t['guarded_input'],**p['scaler'])},
      'g3_state_machine':{'ros__parameters':dict(common,command_topic=t['command'],mission_state_topic=t['mission'],
        battery_topic=t['battery'],navigate_action=t['action'],enable_navigation_transmission=True,require_navigation_handoff=True,
        enable_exploration_completion=p['explorer']['task_completion_enabled'],enable_fault_recovery=True,recovery_scan_topic=t['scan'],recovery_map_frame=f['map'],recovery_robot_frame=f['base'],
        state_publish_period_sec=m['heartbeat_period_s'],recovery_stale_sec=m['recovery_stale_s'],
        recovery_stable_sec=m['recovery_stable_s'],recovery_timeout_sec=m['recovery_timeout_s'],
        low_battery_threshold=m['low_battery_threshold'],home_frame=f['map'],home_x=m['home_x'],home_y=m['home_y'],home_yaw=m['home_yaw'])},
      'vision_explorer':{'ros__parameters':dict(common,require_mission_state=True,use_mission_brain=d['candidate_selector']=='deepseek',
        goal_turn_clearance_m=float(p['navigation'].get('goal_turn_clearance_m',0.)),
        mission_instruction=d['mission_instruction'],brain_timeout_s=d['brain_timeout'],brain_max_candidates=d['brain_max_candidates'],
        goal_obstacle_clearance_m=max(p['navigation']['inflation_radius_m'],p['navigation']['robot_radius_m']+p['guard']['clearance_margin_m']+p['guard']['prediction_speed_mps']*p['guard']['prediction_horizon_s']),map_topic=t['map'],planning_map_topic=t['planning_map'],planning_map_padding_m=max(1.,p['navigation']['robot_radius_m']+p['navigation']['inflation_radius_m']),mission_state_topic=t['mission'],nav_action=t['action'],map_frame=f['map'],robot_frame=f['base'],**p['explorer'])},
    }
    if runtime_root is not None:
        result['vision_explorer']['ros__parameters']['scan_bt_path']=str(Path(runtime_root).resolve()/'scan_bt.xml')
        if p.get('return_localization',{}).get('enabled',False):
            result['g3_state_machine']['ros__parameters']['return_bt_path']=str(Path(runtime_root).resolve()/'return_bt.xml')
    if p.get('corridor',{}).get('enabled',False):
        result['cmd_vel_scaler']['ros__parameters']['corridor_config']=json.dumps(dict(
            settings=p['corridor'],scan_topic=t['scan'],base_frame=f['base'],mission_topic=t['mission'],
            path_topic=t['action'].rsplit('/',1)[0]+'/plan',
            status_topic=t['guard_status']+'/corridor',sensor_timeout_s=min(.5,p['guard']['sensor_timeout_s']),
            odometry_topic=t['odometry'],odom_frame=f['odom'],
            reverse_retreat=(dict(robot_radius_m=p['navigation']['robot_radius_m'],
                desired_linear_velocity_mps=p['navigation']['desired_linear_velocity_mps'],
                max_extent_m=p['navigation'].get('reverse_retreat_max_extent_m',1.5),
                **{k:p['guard'][k] for k in ('clearance_margin_m','prediction_speed_mps','prediction_horizon_s')})
                if p['navigation'].get('reverse_retreat_enabled',False) else None),
            clearance_escape=(dict(robot_radius_m=p['navigation']['robot_radius_m'],
                **{k:p['guard'][k] for k in ('clearance_margin_m','prediction_speed_mps','prediction_horizon_s')})
                if p['guard'].get('clearance_recovery_enabled',False) else None)))
    if p.get('return_localization',{}).get('enabled',False):
        result['g3_state_machine']['ros__parameters'].update(
            odometry_topic=t['odometry'],odom_frame=f['odom'],
            localization_status_topic='/g3/localization_status',
            return_localization_config=json.dumps(p['return_localization'],sort_keys=True))
    if d['candidate_selector']=='deepseek':
        result['mission_brain']={'ros__parameters':dict(common,mission_state_topic=t['mission'],
            context_topic='/mission/context',candidates_topic='/exploration/candidates',decision_topic='/brain/decision')}
    return result


def direct_navigation_commands(source, topic):
    """Formal Nav2 sends physical velocity requests to the one backend ramp."""
    class Rewrite(ast.NodeTransformer):
        def __init__(self):self.removed=0
        def visit_Call(self,node):
            package=next((k.value for k in node.keywords if k.arg=='package'),None)
            if isinstance(package,ast.Constant) and package.value=='nav2_velocity_smoother':
                self.removed+=1;return None
            return self.generic_visit(node)
        def visit_List(self,node):
            node.elts=[v for v in node.elts if not (isinstance(v,ast.Constant) and v.value=='velocity_smoother')]
            return self.generic_visit(node)
        def visit_Tuple(self,node):
            if (len(node.elts)==2 and all(isinstance(v,ast.Constant) for v in node.elts)
                and node.elts[0].value=='cmd_vel' and node.elts[1].value=='cmd_vel_nav'):
                node.elts[1]=ast.Constant(value=topic)
            return self.generic_visit(node)
    rewrite=Rewrite();tree=rewrite.visit(ast.parse(source));ast.fix_missing_locations(tree)
    constants=[n.value for n in ast.walk(tree) if isinstance(n,ast.Constant) and isinstance(n.value,str)]
    if rewrite.removed not in (0,2) or any(v in constants for v in ('nav2_velocity_smoother','velocity_smoother','cmd_vel_nav')):
        raise ValueError('Unexpected Nav2 velocity smoothing launch layout')
    if not any(isinstance(n,ast.Tuple) and len(n.elts)==2 and all(isinstance(v,ast.Constant) for v in n.elts)
               and [v.value for v in n.elts]==['cmd_vel',topic] for n in ast.walk(tree)):
        raise ValueError('Missing direct Nav2 command remapping')
    header=[]
    for line in source.splitlines():
        if not line.strip() or line.lstrip().startswith('#'):header.append(line)
        else:break
    return '\n'.join(header)+'\n'+ast.unparse(tree)+'\n'


def configure_navigation(root,p):
    """Update only generated simulator configs, preserving original source files."""
    root=Path(root);t,f,n=p['topics'],p['frames'],p['navigation']
    path=root/'nav2.yaml';config=yaml.safe_load(path.read_text())
    def replace(v):
        if isinstance(v,dict):return {k:replace(x) for k,x in v.items()}
        if isinstance(v,list):return [replace(x) for x in v]
        if isinstance(v,str):return {'/robot1/velodyne':t['scan'],'/robot1/odometry/filtered':t['odometry'],'map':f['map'],'odom':f['odom'],'base_link':f['base']}.get(v,v)
        return v
    config=replace(config)
    for name in ('local_costmap','global_costmap'):
        params=config[name][name]['ros__parameters'];params['robot_radius']=n['robot_radius_m']
        params['inflation_layer']['inflation_radius']=n['inflation_radius_m']
    config['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic']=t['planning_map']
    config['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']=n['desired_linear_velocity_mps']
    controller=config['controller_server']['ros__parameters']
    # Fixed-gait backend cannot execute an arbitrarily slow final approach.
    # Preserve the original tolerance; independent truth-home acceptance remains .4m.
    controller['general_goal_checker']['xy_goal_tolerance']=.25
    if p.get('return_localization',{}).get('enabled',False):
        import xml.etree.ElementTree as ET
        # Frontier goals may ignore final yaw. RETURN must satisfy the same
        # orientation contract as its subsequent independent home verifier.
        controller['return_goal_checker']=dict(
            plugin='nav2_controller::SimpleGoalChecker',stateful=False,
            xy_goal_tolerance=.25,
            yaw_goal_tolerance=min(.25,p['return_localization']['yaw_tolerance_rad']))
        if 'return_goal_checker' not in controller['goal_checker_plugins']:
            controller['goal_checker_plugins'].append('return_goal_checker')
        tree=ET.parse(root/'nav2_go2_bt.xml')
        for follow_path in tree.findall('.//FollowPath'):
            follow_path.set('goal_checker_id','return_goal_checker')
            follow_path.set('controller_id','ReturnFollowPath')
        tree.write(root/'return_bt.xml',encoding='utf-8',xml_declaration=True)
    follow=controller['FollowPath']
    follow.update(lookahead_dist=.35,min_lookahead_dist=.25,max_lookahead_dist=.5,
                  rotate_to_heading_min_angle=math.pi/2,rotate_to_heading_angular_vel=.4,max_angular_accel=3.2,
                  regulated_linear_scaling_min_speed=n['desired_linear_velocity_mps'],
                  min_approach_linear_velocity=n['desired_linear_velocity_mps'])
    if p.get('return_localization',{}).get('enabled',False):
        # A fixed-speed approach with saturated yaw can orbit a nearby goal.
        # Align sooner on RETURN, using another plugin in the same server and
        # preserving the frontier controller, gait speed and home tolerances.
        controller['ReturnFollowPath']=copy.deepcopy(follow)
        controller['ReturnFollowPath']['plugin']='qwen_robotnav_progress::ReturnController'
        controller['ReturnFollowPath']['rotate_to_heading_min_angle']=.5
        controller['ReturnFollowPath']['terminal_turn_handoff_enabled']=n.get('return_turn_handoff_enabled',False)
        if 'ReturnFollowPath' not in controller['controller_plugins']:
            controller['controller_plugins'].append('ReturnFollowPath')
    config.pop('velocity_smoother',None)
    planner=config['planner_server']['ros__parameters']['GridBased']
    planner.update(tolerance=.05,allow_unknown=False)
    behavior=config['behavior_server']['ros__parameters']
    behavior.update(max_rotational_vel=.4,min_rotational_vel=.15,rotational_acc_lim=.6)
    if 'scan_goal_checker' not in controller['goal_checker_plugins']:
        controller['goal_checker_plugins'].append('scan_goal_checker')
    controller['scan_goal_checker']=dict(plugin='nav2_controller::SimpleGoalChecker',stateful=True,
                                         xy_goal_tolerance=.2,yaw_goal_tolerance=.15)
    (root/'scan_bt.xml').write_text('''<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree"><Sequence name="BoundedInPlaceScan">
    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
    <FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="scan_goal_checker"/>
    <Wait wait_duration="0.5"/>
  </Sequence></BehaviorTree>
</root>
''')
    path.write_text(yaml.safe_dump(config,sort_keys=False))
    path=root/'slam.yaml';slam=yaml.safe_load(path.read_text())['slam_toolbox']['ros__parameters']
    slam.update(scan_topic=t['scan'],base_frame=f['base'],odom_frame=f['odom'],map_frame=f['map'])
    path.write_text(yaml.safe_dump({'slam_toolbox':{'ros__parameters':slam}},sort_keys=False))
    for name in ('navigation.launch.py','slam_navigation.launch.py'):
        path=root/name;s=path.read_text()
        for old,key in [('/robot1/tf_static','tf_static'),('/robot1/tf','tf'),('/robot1/map','map'),('/deepseek/nav_cmd','nav_command')]:s=s.replace(repr(old),repr(t[key]))
        if name=='navigation.launch.py':s=direct_navigation_commands(s,t['nav_command'])
        path.write_text(s)


def expected_runtime_roles(profile):
    roles={'guard','battery','scaler','g3','navigation','frontier'}
    if profile['decision']['candidate_selector']=='deepseek':roles.add('mission_brain')
    return roles


def runtime_exit_ok(info,roles):
    codes=info.get('child_exit_codes',{})
    return bool(info.get('exit_code')==0 and set(codes)==set(roles)
                and all(type(code) is int and code==0 for code in codes.values()))
