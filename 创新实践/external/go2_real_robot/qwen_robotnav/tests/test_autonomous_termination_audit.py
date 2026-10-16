"""A successful home action alone cannot prove the runtime ended exploration itself."""
import importlib.util
from pathlib import Path
import pytest
pytest.importorskip('rclpy')
spec=importlib.util.spec_from_file_location('runtime_termination_audit',Path(__file__).parents[1]/'scripts/evaluate_runtime.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def evidence():
    result=dict(session_id='a'*32,reason='time_budget',completed=False)
    return [result],[dict(result)],['explore','estop'],['exploration:time_budget']


def test_matching_session_and_runtime_trigger_with_no_external_return_pass():
    assert audit.autonomous_termination_ok(*evidence())


@pytest.mark.parametrize('change', ['manual_return','wrong_session','missing_request','wrong_trigger','false_completion','multiple_results'])
def test_autonomy_claim_rejected_when_evidence_does_not_match(change):
    results,requests,commands,triggers=evidence()
    if change=='manual_return': commands.append('return')
    elif change=='wrong_session': requests[0]['session_id']='old'
    elif change=='missing_request': requests.clear()
    elif change=='wrong_trigger': triggers[:]=['command']
    elif change=='false_completion': results[0]['completed']=True;requests[0]['completed']=True
    elif change=='multiple_results': results.append(dict(results[0],session_id='other'))
    assert not audit.autonomous_termination_ok(results,requests,commands,triggers)
