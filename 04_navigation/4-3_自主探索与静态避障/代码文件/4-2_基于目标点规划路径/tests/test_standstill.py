import math
from lite3_lab_5_2.standstill import PoseStandstill


def feed(function):
    estimator=PoseStandstill()
    for i in range(31):
        t=i*.1;x,y,yaw=function(t,i)
        estimator.update(t,x,y,yaw)
    return estimator


def test_fixed_pose_with_difference_noise_can_confirm_standstill():
    estimator=feed(lambda t,i:(.003*(-1)**i,.002*(-1)**i,.003*(-1)**i))
    result=estimator.report(3.02)
    # Adjacent samples suggest 0.072m/s although the pose only oscillates 7mm.
    assert result['stationary']
    assert result['position_span_m']<.01
    assert result['fitted_speed_mps']<=.005


def test_constant_slow_translation_and_rotation_are_not_stationary():
    assert not feed(lambda t,i:(.01*t,0.,0.)).report(3.02)['stationary']
    assert not feed(lambda t,i:(0.,0.,.02*t)).report(3.02)['stationary']


def test_returning_to_same_pose_does_not_hide_an_excursion():
    estimator=feed(lambda t,i:(.05*math.sin(math.pi*t),0.,0.))
    result=estimator.report(3.02)
    assert result['position_span_m']>.09
    assert not result['stationary']


def test_yaw_wrap_does_not_look_like_a_full_rotation():
    estimator=feed(lambda t,i:(0.,0.,(math.pi+.003*(-1)**i+math.pi)%(2*math.pi)-math.pi))
    result=estimator.report(3.02)
    assert result['stationary'] and result['yaw_span_rad']<.01


def test_missing_duplicate_backward_or_nonfinite_data_invalidates_window():
    for stamp in [3.,2.,4.]:
        estimator=feed(lambda t,i:(0.,0.,0.))
        estimator.update(stamp,0.,0.,0.)
        assert not estimator.report(stamp)['ready']
    estimator=feed(lambda t,i:(0.,0.,0.))
    assert not estimator.report(4.1)['stationary']
    estimator.update(3.1,float('nan'),0.,0.)
    assert not estimator.report(3.2)['ready']


def test_new_window_must_cover_two_seconds():
    estimator=PoseStandstill()
    for i in range(20):estimator.update(i*.1,0.,0.,0.)
    assert not estimator.report(1.91)['ready']
    estimator.update(2.,0.,0.,0.)
    assert estimator.report(2.01)['stationary']
