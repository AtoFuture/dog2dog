"""Static Gazebo fixtures and independent geometric acceptance; never a navigation input."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

SCENARIOS = ('narrow', 'dead_end', 'unreachable')


def scene_spec(name):
    # Each box is [center_x, center_y, length_x, length_y]; height is 1.5m.
    if name == 'narrow':
        boxes = [[3., -.775, 10.3, .15], [3., .775, 10.3, .15],
                 [-2.075, 0., .15, 1.4], [8.075, 0., .15, 1.4]]
        return dict(scenario=name, boxes=boxes, clear_width_m=1.4,
                    visit_region=[4., 7.8, -.6, .6], forbidden_region=None,
                    min_valid_goals=1, max_frontier_attempts=None,
                    end_reasons=['no_frontiers', 'candidates_unavailable'])
    if name == 'dead_end':
        boxes = [[-2.075, 0., .15, 4.3], [-.5, -2.075, 3., .15],
                 [-.5, 2.075, 3., .15], [1.075, -1.45, .15, 1.1],
                 [1.075, 1.45, .15, 1.1], [3.65, - .975, 5.3, .15],
                 [3.65, .975, 5.3, .15], [6.075, 0., .15, 1.8]]
        return dict(scenario=name, boxes=boxes, clear_width_m=1.8,
                    visit_region=[2., 5.8, -.8, .8], forbidden_region=None,
                    min_valid_goals=1, max_frontier_attempts=None,
                    end_reasons=['no_frontiers', 'candidates_unavailable'])
    if name == 'unreachable':
        # A 0.30m slit admits lidar rays but cannot admit the 0.80m footprint.
        boxes = [[3., -2.075, 10.3, .15], [3., 2.075, 10.3, .15],
                 [-2.075, 0., .15, 4.], [8.075, 0., .15, 4.],
                 [3.075, -1.075, .15, 1.85], [3.075, 1.075, .15, 1.85]]
        return dict(scenario=name, boxes=boxes, clear_width_m=.3,
                    visit_region=None, forbidden_region=[3.3, 8., -2., 2.],
                    min_valid_goals=0, max_frontier_attempts=None, max_inaccessible_targets=0,
                    end_reasons=['candidates_unavailable', 'no_frontiers'])
    raise ValueError('Unknown static scenario: '+str(name))


def _box(world, name, box):
    x, y, sx, sy = box
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x} {y} .75 0 0 0'
    link = ET.SubElement(model, 'link', name='wall')
    for kind in ('collision', 'visual'):
        item = ET.SubElement(link, kind, name='wall')
        ET.SubElement(ET.SubElement(ET.SubElement(item, 'geometry'), 'box'), 'size').text = f'{sx} {sy} 1.5'
        if kind == 'visual':
            material = ET.SubElement(item, 'material')
            ET.SubElement(material, 'ambient').text = '.65 .65 .65 1'
            ET.SubElement(material, 'diffuse').text = '.65 .65 .65 1'


def prepare(world_path, output, scenario):
    spec = scene_spec(scenario)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    if world is None or world.get('name') != 'world_demo' or world.find("model[@name='ground_plane']") is None:
        raise ValueError('Require simulator world_demo and its ground plane')
    # Preserve the simulator physics, sensor/scene plugins and flat ground.
    # Remove warehouse geometry only in this private copy.
    for item in list(world):
        if item.tag == 'include' or (item.tag == 'model' and item.get('name') != 'ground_plane'):
            world.remove(item)
    for i, box in enumerate(spec['boxes']):
        _box(world, f'static_trial_wall_{i}', box)
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '.6 .6 .6 1'
    light = ET.SubElement(world, 'light', type='directional', name='static_trial_sun')
    ET.SubElement(light, 'direction').text = '-.3 -.3 -1'
    ET.SubElement(light, 'diffuse').text = '.8 .8 .8 1'
    target = output/'static_world.sdf'
    tree.write(target, encoding='utf-8', xml_declaration=True)
    metadata = dict(schema_version=1, **spec, evaluation_only=True, spawn_world_xy=[0.,0.],
                    world_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                    source_world_sha256=hashlib.sha256(Path(world_path).read_bytes()).hexdigest())
    (output/'static_scene.json').write_text(json.dumps(metadata, indent=2))
    return target


def load_scene(root):
    root = Path(root)
    metadata = json.loads((root/'static_scene.json').read_text())
    spec = scene_spec(metadata['scenario'])
    if metadata.get('schema_version') != 1 or metadata.get('evaluation_only') is not True or metadata.get('spawn_world_xy') != [0.,0.]:
        raise ValueError('Invalid static scene metadata')
    if any(metadata.get(k) != v for k, v in spec.items()):
        raise ValueError('Static scene contract has been altered')
    if hashlib.sha256((root/'static_world.sdf').read_bytes()).hexdigest() != metadata['world_sha256']:
        raise ValueError('Static world differs from recorded fixture')
    return metadata


def point_box_distance(x, y, box):
    cx, cy, sx, sy = box
    dx, dy = abs(x-cx)-sx/2, abs(y-cy)-sy/2
    return math.hypot(max(dx, 0.), max(dy, 0.)) if max(dx, dy) >= 0 else max(dx, dy)


def _inside(x, y, region):
    return region[0] <= x <= region[1] and region[2] <= y <= region[3]


def evaluate_scene(scene, body, events, results, *, robot_radius, world_from_map=None):
    """body: mission-only (wall_time,x,y,z,roll,pitch,yaw), from recorded GT."""
    finite = bool(body) and all(all(math.isfinite(v) for v in p) for p in body)
    clearance = min((point_box_distance(p[1], p[2], box) for p in body for box in scene['boxes']), default=-math.inf)
    attempted = sum(e.get('status') == 'sending' and e.get('purpose','frontier') == 'frontier' for e in events)
    scans = sum(e.get('status') == 'sending' and e.get('purpose') == 'scan' for e in events)
    unavailable = any(e.get('status') == 'search_unavailable' and e.get('reason') in ('NO_CANDIDATE','SMALL_FRONTIERS') for e in events)
    checks = dict(static_body_evidence=finite, static_bounded_scans=scans<=3,
                  static_geometric_clearance=finite and clearance >= robot_radius-.03,
                  static_upright=finite and all(.15 <= p[3] <= .4 and max(abs(p[4]), abs(p[5])) < .6 for p in body),
                  static_expected_termination=bool(results) and results[0].get('reason') in scene['end_reasons'])
    if scene['visit_region'] is not None:
        checks['static_region_reached'] = finite and any(_inside(p[1], p[2], scene['visit_region']) for p in body)
    if scene['forbidden_region'] is not None:
        checks['static_inaccessible_region_avoided'] = finite and not any(_inside(p[1], p[2], scene['forbidden_region']) for p in body)
    inaccessible_targets=0
    if scene.get('max_inaccessible_targets') is not None:
        for event in events:
            if event.get('status')!='sending' or event.get('purpose','frontier')!='frontier':continue
            target=event.get('target',{})
            try:
                point=compose_pose(world_from_map,(float(target['x']),float(target['y']),0.))
                if not all(math.isfinite(v) for v in point) or _inside(point[0],point[1],scene['forbidden_region']):
                    inaccessible_targets+=1
            except (KeyError,TypeError,ValueError):inaccessible_targets+=1
        checks['static_unreachable_goals_rejected'] = inaccessible_targets<=scene['max_inaccessible_targets'] and unavailable
    scan_motion=[]
    for event in events:
        if event.get('status')!='sending' or event.get('purpose')!='scan':continue
        terminal=next((e for e in events if e.get('attempt_id')==event.get('attempt_id')
                       and e.get('status') in ('succeeded','aborted','canceled','rejected')),None)
        begin,end=event.get('recorded_time'),terminal.get('recorded_time') if terminal else None
        samples=[p for p in body if len(p)>=7 and begin is not None and end is not None and begin-.05<=p[0]<=end+.05]
        if len(samples)<2:continue
        rotation=max(abs(math.atan2(math.sin(p[6]-samples[0][6]),math.cos(p[6]-samples[0][6]))) for p in samples)
        drift=max(math.dist(p[1:3],samples[0][1:3]) for p in samples)
        scan_motion.append(dict(attempt_id=event.get('attempt_id'),rotation_rad=rotation,translation_m=drift,
                                succeeded=terminal['status']=='succeeded'))
    if scans:
        checks['static_real_scan_motion']=any(e['succeeded'] and e['rotation_rad']>=.2 and e['translation_m']<=.3 for e in scan_motion)
    return dict(checks=checks, scan_motion=scan_motion, minimum_body_wall_distance_m=clearance if math.isfinite(clearance) else None,
                frontier_attempts=attempted, inaccessible_target_attempts=inaccessible_targets, scan_attempts=scans, scenario=scene['scenario'],
                limitation='Ground-truth circle clearance is an independent geometric proxy, not a contact-sensor measurement.')


def scan_wall_hits(scene, scan, pose, sensor):
    """Match real measured returns to the fixture walls using evaluator-only GT."""
    x, y, heading = pose
    sx, sy, sa = sensor
    ox=x+math.cos(heading)*sx-math.sin(heading)*sy
    oy=y+math.sin(heading)*sx+math.cos(heading)*sy
    hits=[0]*len(scene['boxes'])
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min < distance < scan.range_max:
            continue
        angle=heading+sa+scan.angle_min+i*scan.angle_increment
        px,py=ox+distance*math.cos(angle),oy+distance*math.sin(angle)
        for index,box in enumerate(scene['boxes']):
            if abs(point_box_distance(px,py,box)) <= .08:
                hits[index]+=1
    return hits


def quaternion_rpy(q):
    roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return roll,pitch,yaw


def configure_spawn(launch_file):
    """Change only the generated launch copy, keeping the shared robot config intact."""
    launch_file=Path(launch_file)
    source=launch_file.read_text()
    needle="    robots = yaml_data['robots']"
    if source.count(needle)!=1:
        raise ValueError('Cannot locate unique robot spawn configuration')
    replacement=needle+"\n    if len(robots) != 1 or robots[0]['name'] != 'robot1':\n        raise ValueError('Static fixtures require only robot1')\n    robots = [dict(robots[0], x_pose='0.0', y_pose='0.0')]"
    launch_file.write_text(source.replace(needle,replacement,1))


def compose_pose(first, second):
    x,y,a=first;u,v,b=second
    return (x+math.cos(a)*u-math.sin(a)*v,y+math.sin(a)*u+math.cos(a)*v,a+b)


def inverse_pose(pose):
    x,y,a=pose
    return (-math.cos(a)*x-math.sin(a)*y,math.sin(a)*x-math.cos(a)*y,-a)
