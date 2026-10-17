"""Repeated cleanup signals must leave a failure report instead of interrupting finally."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import pytest
pytest.importorskip('rclpy')
ROOT=Path(__file__).parents[1]


def test_repeated_stop_signals_preserve_observer_failure_report(tmp_path):
    env=dict(os.environ,IGN_PARTITION='runtime_shutdown_'+uuid.uuid4().hex[:8])
    domain=env.get('ROS_DOMAIN_ID','231')
    command=[sys.executable,str(ROOT/'scripts/validate_runtime.py'),'--profile',str(ROOT/'config/sim_runtime.yaml'),
             '--output-dir',str(tmp_path),'--expected-domain',domain]
    with (tmp_path/'observer.log').open('w') as log:
        child=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+10
            while time.monotonic()<deadline and not (tmp_path/'gt_bridge.log').exists() and child.poll() is None:time.sleep(.05)
            assert (tmp_path/'gt_bridge.log').exists()
            child.send_signal(signal.SIGINT)
            time.sleep(.2)
            if child.poll() is None:child.send_signal(signal.SIGTERM)
            assert child.wait(timeout=15)==1
            report=json.loads((tmp_path/'report.json').read_text())
            assert report['success'] is False and report['recording_complete'] is False
            assert report['operator_commands']==['ESTOP']
            assert 'Traceback' not in (tmp_path/'observer.log').read_text()
        finally:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGKILL);child.wait()


def test_empty_map_cannot_mark_readonly_observer_ready():
    import importlib.util
    from types import SimpleNamespace
    spec=importlib.util.spec_from_file_location('runtime_map_readiness',ROOT/'scripts/validate_runtime.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    node=SimpleNamespace(latest_map=None,first_area=None)
    bad=SimpleNamespace(info=SimpleNamespace(width=0,height=0,resolution=.05),data=[])
    module.Observer.mapping(node,bad)
    assert node.latest_map is None and node.first_area is None
    good=SimpleNamespace(info=SimpleNamespace(width=2,height=2,resolution=.05),data=[0,0,-1,100])
    module.Observer.mapping(node,good)
    assert node.latest_map is good and node.first_area==pytest.approx(.0075)


def test_home_reference_uses_localized_map_origin_instead_of_startup_sample():
    import importlib.util
    import math
    from types import SimpleNamespace as NS
    spec=importlib.util.spec_from_file_location('runtime_home_reference',ROOT/'scripts/validate_runtime.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    # Body has drifted 0.09 m from map home and faces north in world.
    transform=NS(translation=NS(x=.09,y=0),rotation=NS(x=0,y=0,z=0,w=1))
    node=NS(profile={'frames':{'map':'map','base':'base'},'mission':{'home_x':0,'home_y':0,'home_yaw':0}},
            truth_pose=(time.monotonic(),7.,6.09,math.pi/2),
            tf=NS(lookup_transform=lambda *args:NS(transform=transform)))
    assert module.Observer.register_home(node)==pytest.approx((7.,6.,math.pi/2))
    node.truth_pose=(time.monotonic()-1,7.,6.09,math.pi/2)
    with pytest.raises(RuntimeError,match='Fresh truth pose'):
        module.Observer.register_home(node)
