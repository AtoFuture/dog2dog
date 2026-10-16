"""A lost simulator RPC must neither abort immediately nor replay stale placement."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
script=Path(__file__).resolve().parents[1]/'scripts/drive_dynamic_cart.py'
spec=importlib.util.spec_from_file_location('cart_retry',script)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_lost_ack_retries_same_request_after_refreshing_safety():
    calls=[];checks=[];retries=[]
    def run(command,**kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0,stdout='data: true' if len(calls)>1 else '',stderr='')
    module.set_cart_pose('fixed scoped pose',before_attempt=checks.append,on_retry=lambda:retries.append(True),run=run)
    assert len(calls)==2 and calls[0]==calls[1] and checks==[0,1] and len(retries)==1


def test_failed_pose_service_has_three_attempt_limit():
    calls=[]
    def run(command,**kwargs):calls.append(command);return SimpleNamespace(returncode=1,stdout='',stderr='timeout')
    with pytest.raises(RuntimeError,match='after 3 attempts'):
        module.set_cart_pose('pose',before_attempt=lambda i:None,on_retry=lambda:None,run=run)
    assert len(calls)==3


def test_retry_aborts_before_mutation_if_refreshed_robot_pose_is_unsafe():
    calls=[]
    def run(command,**kwargs):calls.append(command);return SimpleNamespace(returncode=1,stdout='',stderr='timeout')
    def check(attempt):
        if attempt:raise RuntimeError('unsafe placement')
    with pytest.raises(RuntimeError,match='unsafe'):
        module.set_cart_pose('pose',before_attempt=check,on_retry=lambda:None,run=run)
    assert len(calls)==1
