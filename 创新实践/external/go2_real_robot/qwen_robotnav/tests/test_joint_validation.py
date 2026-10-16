"""A successful return cannot mask failed concurrent tracking or reacquisition."""
import importlib.util
from pathlib import Path
import sys
import pytest
pytest.importorskip('rclpy')
scripts=Path(__file__).resolve().parents[1]/'scripts'
with pytest.MonkeyPatch.context() as patch:
    patch.syspath_prepend(str(scripts))
    spec=importlib.util.spec_from_file_location('joint_validation',scripts/'evaluate_joint_recon.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


@pytest.mark.parametrize('failure',['none','identity','dynamic','return','motion','overlap','reacquisition'])
def test_all_joint_components_and_post_cart_evidence_are_required(failure):
    report=dict(success=True);moving=dict(success=failure!='identity',identity_validation_success=failure!='identity',
        target_audits=[dict(name='p1'),dict(name='p2')],audits=[dict(truth_name=name,stamp_ns=20)
            for name in ['p1','p1','p2',*([] if failure=='reacquisition' else ['p2'])]])
    result=module.combine(report,moving,dict(success=failure!='return'),dict(success=failure!='dynamic'),
        dict(wait_for_robot_motion=True,success=True),dict(end_stamp_ns=10),
        .49 if failure=='motion' else .5,9.9 if failure=='overlap' else 10.)
    assert result['success']==(failure=='none')
