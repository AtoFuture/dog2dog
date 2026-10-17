import copy
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import pytest
from qwen_robotnav.static_scenarios import prepare,load_scene,scene_spec,evaluate_scene,scan_wall_hits


def test_private_world_preserves_physics_and_has_matching_visible_collision_walls(tmp_path):
    source=tmp_path/'warehouse.sdf'
    text='<sdf><world name="world_demo"><physics/><plugin name="sensors"/><model name="ground_plane"/><include><uri>warehouse</uri></include><model name="depot_collision"/></world></sdf>'
    source.write_text(text)
    path=prepare(source,tmp_path/'trial','narrow')
    assert source.read_text()==text
    world=ET.parse(path).getroot().find('world')
    assert world.find('physics') is not None and world.find('plugin') is not None
    assert world.find('include') is None and world.find("model[@name='depot_collision']") is None
    walls=world.findall("model/link/collision/geometry/box/size")
    visuals=world.findall("model/link/visual/geometry/box/size")
    assert len(walls)==len(visuals)==4
    assert [n.text for n in walls]==[n.text for n in visuals]
    assert load_scene(tmp_path/'trial')['clear_width_m']==1.4
    path.write_text(path.read_text()+' ')
    with pytest.raises(ValueError,match='differs'):load_scene(tmp_path/'trial')


def body(x,y=0.,z=.23,yaw=0.,at=1.):return (at,x,y,z,0.,0.,yaw)


def test_dead_end_requires_entering_the_branch_and_clearance():
    scene=scene_spec('dead_end');result=[dict(reason='no_frontiers')]
    assert not all(evaluate_scene(scene,[body(0)],[],result,robot_radius=.4)['checks'].values())
    assert all(evaluate_scene(scene,[body(0),body(3),body(0)],[],result,robot_radius=.4)['checks'].values())
    checks=evaluate_scene(scene,[body(0),body(3,.8)],[],result,robot_radius=.4)['checks']
    assert not checks['static_geometric_clearance']


@pytest.mark.parametrize('missing',['body','refusal','unsafe_goal','forbidden','budget'])
def test_unreachable_scene_must_refuse_goals_without_claiming_movement(missing):
    scene=scene_spec('unreachable');samples=[body(0),body(.1)]
    events=[dict(status='search_unavailable',reason='NO_CANDIDATE')]
    results=[dict(reason='candidates_unavailable')]
    if missing=='body':samples=[]
    if missing=='refusal':events=[]
    if missing=='unsafe_goal':events.append(dict(status='sending'))
    if missing=='forbidden':samples.append(body(4))
    if missing=='budget':results=[dict(reason='time_budget')]
    assert not all(evaluate_scene(scene,samples,events,results,robot_radius=.4)['checks'].values())


def test_real_lidar_returns_must_hit_fixture_geometry():
    scene=scene_spec('narrow')
    scan=SimpleNamespace(ranges=[.7,.7,.7],angle_min=1.5707963268,angle_increment=0.,range_min=.1,range_max=10.)
    assert scan_wall_hits(scene,scan,(0.,0.,0.),(0.,0.,0.))[1]==3
    scan.ranges=[3.,3.,3.]
    assert not any(scan_wall_hits(scene,scan,(0.,0.,0.),(0.,0.,0.)))


def test_both_navigable_passages_fit_the_configured_footprint():
    assert scene_spec('narrow')['clear_width_m']>2*.45
    assert scene_spec('dead_end')['clear_width_m']>2*.45
    assert scene_spec('unreachable')['clear_width_m']<2*.4


def test_private_static_launch_overrides_warehouse_spawn_without_changing_height(tmp_path):
    from qwen_robotnav.static_scenarios import configure_spawn
    launch=tmp_path/'private_launch.py'
    launch.write_text("def robot():\n    robots = yaml_data['robots']\n    return robots\n")
    configure_spawn(launch)
    scope={'yaml_data':{'robots':[dict(name='robot1',x_pose='7.0',y_pose='6.0',z_pose='0.4')]}}
    exec(launch.read_text(),scope)
    assert scope['robot']()==[dict(name='robot1',x_pose='0.0',y_pose='0.0',z_pose='0.4')]
    assert scope['yaml_data']['robots'][0]['x_pose']=='7.0'
    scope['yaml_data']['robots'].append(dict(name='robot2'))
    with pytest.raises(ValueError,match='only robot1'):scope['robot']()


def test_unreachable_scene_separates_safe_in_place_scans_from_frontier_goals():
    scene=scene_spec('unreachable')
    scans=[dict(status='sending',purpose='scan',attempt_id=i,recorded_time=1.) for i in range(3)]
    events=scans+[dict(status='succeeded',purpose='scan',attempt_id=i,recorded_time=3.) for i in range(3)]+[dict(status='search_unavailable',reason='NO_CANDIDATE')]
    samples=[body(0),body(.01,yaw=.6,at=3.)]
    result=evaluate_scene(scene,samples,events,[dict(reason='candidates_unavailable')],robot_radius=.4)
    assert all(result['checks'].values())
    assert result['scan_attempts']==3 and result['frontier_attempts']==0
    events.append(dict(status='sending',purpose='scan'))
    assert not evaluate_scene(scene,samples,events,[dict(reason='candidates_unavailable')],robot_radius=.4)['checks']['static_bounded_scans']


def test_nominal_scan_success_without_real_yaw_motion_is_rejected():
    scene=scene_spec('narrow')
    events=[dict(status='sending',purpose='scan',attempt_id=1,recorded_time=1.),
            dict(status='succeeded',purpose='scan',attempt_id=1,recorded_time=3.)]
    samples=[body(0),body(0,at=3.)]
    result=evaluate_scene(scene,samples,events,[dict(reason='candidates_unavailable')],robot_radius=.4)
    assert not result['checks']['static_real_scan_motion']


def test_unreachable_room_allows_local_goals_but_rejects_targets_beyond_slit():
    scene=scene_spec('unreachable');results=[dict(reason='candidates_unavailable')]
    events=[dict(status='search_unavailable',reason='NO_CANDIDATE'),
            dict(status='sending',purpose='frontier',target=dict(x=1.,y=0.))]
    assert all(evaluate_scene(scene,[body(0),body(1)],events,results,robot_radius=.4,world_from_map=(0,0,0))['checks'].values())
    events[-1]['target']['x']=4.
    assert not evaluate_scene(scene,[body(0)],events,results,robot_radius=.4,world_from_map=(0,0,0))['checks']['static_unreachable_goals_rejected']
    assert not evaluate_scene(scene,[body(0)],events,results,robot_radius=.4)['checks']['static_unreachable_goals_rejected']
