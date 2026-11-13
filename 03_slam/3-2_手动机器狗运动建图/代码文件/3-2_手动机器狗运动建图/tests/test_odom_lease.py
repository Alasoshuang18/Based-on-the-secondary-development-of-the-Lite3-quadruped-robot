"""Preserve real stamps and ordering while accepting the requested 1s lease."""
from types import SimpleNamespace as NS
from test_exploration_lifecycle import method
from lite3_lab_5_1.safety import fresh_source


def test_delayed_measurement_within_one_second_updates_but_old_or_replayed_data_does_not():
    receive=method('on_odom',dict(time=NS(monotonic=lambda:50.),fresh_source=fresh_source,
        Time=NS(from_msg=lambda stamp:stamp),math=__import__('math'),TransformException=RuntimeError))
    rows=[];events=[]
    node=NS(odom_stamp=None,sim_now=lambda:100.,event=lambda *a,**k:events.append((a,k)),
        estimate_xyyaw=lambda stamp:(0.,0.,0.),estimate_csv=NS(writerow=rows.append),
        estimate_file=NS(flush=lambda:None),motion=NS(observe=lambda *a:None),
        standstill=NS(observe_message=lambda *a:None),upper={},upper_at=50.)
    message=NS(header=NS(stamp=NS(sec=99,nanosec=200000000)),twist=NS(twist=NS(
        linear=NS(x=0.,y=0.),angular=NS(z=0.))))
    receive(node,message)
    assert node.odom_stamp==99.2 and len(rows)==1
    receive(node,message)
    assert len(rows)==1 and events
    message.header.stamp=NS(sec=98,nanosec=900000000)
    receive(node,message)
    assert node.odom_stamp==99.2 and len(rows)==1
    message.header.stamp=NS(sec=100,nanosec=100000000)
    receive(node,message)
    assert node.odom_stamp==99.2
