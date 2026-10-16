"""The simulator's fault must withhold actuation, never add motion or bypass safety."""
import importlib.util
import json
from pathlib import Path
import time
from types import SimpleNamespace,MethodType
import pytest
pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist

script=Path(__file__).resolve().parents[1]/'scripts/validate_slam_frontier.py'
@pytest.fixture(scope='module')
def validation_module():
    project=script.parents[4]
    with pytest.MonkeyPatch.context() as patch:
        for relative in ('baseline/g3_integration_eval','baseline/g2_vision_explore',
                         'baseline/g2_vision_explore/vision_explorer'):
            patch.syspath_prepend(str(project/relative))
        spec=importlib.util.spec_from_file_location('return_hold_validation',script)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def monitor(module):
    clock=SimpleNamespace(nanoseconds=500_000_000)
    emitted=[];events=[];now=time.monotonic()
    command=Twist();command.linear.x=.03;command.angular.z=.5
    fake=SimpleNamespace(return_hold_start=0.,return_hold_s=1.,return_hold_released=False,
        held_control_cycles=0,fault_events=[],running=True,command=command,command_at=now,
        scan_at=now,poses=[(now,0,0,.23,0)],front=2.,blocked=0,forwarded=0,
        get_clock=lambda:SimpleNamespace(now=lambda:clock),
        pub=SimpleNamespace(publish=emitted.append),fault_pub=SimpleNamespace(publish=events.append))
    fake._fault=MethodType(module.Monitor._fault,fake)
    fake._holding_return=MethodType(module.Monitor._holding_return,fake)
    return fake,clock,emitted,events


def test_fault_outputs_zero_then_releases_exactly_once(validation_module):
    module=validation_module
    fake,clock,emitted,events=monitor(module)
    module.Monitor.control(fake)
    assert emitted[-1].linear.x==emitted[-1].angular.z==0 and fake.forwarded==0
    clock.nanoseconds=1_100_000_000
    module.Monitor.control(fake);module.Monitor.control(fake)
    assert emitted[-1].linear.x==.03 and emitted[-1].angular.z==.5
    assert fake.return_hold_released and len(events)==1
    assert json.loads(events[0].data)['status']=='return_hold_released'


def test_releasing_fault_preserves_obstacle_and_stale_command_guards(validation_module):
    module=validation_module
    fake,clock,emitted,_=monitor(module);clock.nanoseconds=1_100_000_000
    fake.front=.1;module.Monitor.control(fake)
    assert emitted[-1].linear.x==0 and fake.blocked==1
    fake.front=2.;fake.command_at-=2.;module.Monitor.control(fake)
    assert emitted[-1].linear.x==emitted[-1].angular.z==0
    fake.command_at=time.monotonic();fake.running=False;module.Monitor.control(fake)
    assert emitted[-1].linear.x==emitted[-1].angular.z==0
