import importlib.util
import json
from pathlib import Path

script=Path(__file__).resolve().parents[1]/'scripts/run_return_regression.py'
spec=importlib.util.spec_from_file_location('return_regression',script)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_return_pass_does_not_hide_failed_identity_or_missing_fault(tmp_path):
    for name,_,hold in module.CASE_SPECS:
        root=tmp_path/name;root.mkdir()
        for filename,data in {
            'report.json':dict(return_hold_s=hold),
            'moving_validation.json':dict(success=name!='crossing',identity_validation_success=name!='crossing'),
            'return_recovery_validation.json':dict(success=True,return_success=True,hold_injection_success=True if hold else None),
        }.items():(root/filename).write_text(json.dumps(data))
    result=module.summarize(tmp_path,221,dict(crossing=1,parallel=0,return_hold=0))
    assert result['return_regression_success'] and not result['full_mission_success']
    assert result['cases'][0]['identity_success'] is False
    # A normal successful RETURN cannot stand in for the required injected fault.
    p=tmp_path/'return_hold/return_recovery_validation.json';p.write_text(json.dumps(dict(success=True,return_success=True)))
    assert not module.summarize(tmp_path,221)['return_regression_success']
