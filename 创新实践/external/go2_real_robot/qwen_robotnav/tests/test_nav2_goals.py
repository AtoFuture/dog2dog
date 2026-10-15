import math
import numpy as np
import pytest
from qwen_robotnav.nav2_goals import PlanarGoal, select_goal, transform_goal


def test_uses_distant_metric_goal_beyond_nearby_obstacle():
    goal = select_goal([[0.5,0,0],[1.6,0,0],[2.47,0,0],[2.93,0,0]])
    assert goal == PlanarGoal(2.47,0,0)


def test_short_or_empty_prediction_does_not_create_nav2_goal():
    assert select_goal([]) is None
    assert select_goal([[0.5,0,0]]) is None


def test_capture_pose_transform_preserves_metric_scale_and_rotates_left():
    goal = transform_goal(PlanarGoal(2,0,0.2),7,6,math.pi/2)
    np.testing.assert_allclose([goal.x,goal.y,goal.yaw],[7,8,math.pi/2+0.2])


@pytest.mark.parametrize('limits',[(0,2),(2,1),(1,float('inf'))])
def test_invalid_goal_limits_fail(limits):
    with pytest.raises(ValueError):select_goal([], *limits)


def test_nonfinite_prediction_is_rejected():
    with pytest.raises(ValueError):select_goal([[float('nan'),0,0]])
