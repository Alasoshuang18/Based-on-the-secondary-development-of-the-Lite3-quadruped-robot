"""在没有ROS的主机检查所有第三章节点是否覆盖Humble Node只读属性。"""
import ast
from pathlib import Path


def test_course_nodes_do_not_assign_readonly_rclpy_node_properties():
    readonly = {'publishers','subscriptions','timers','services','clients','guards',
                'waitables','context','handle','default_callback_group'}
    root = Path(__file__).resolve().parents[2]
    failures = []
    sources = list(root.glob('*/lite3_lab_*/experiment.py'))
    assert sources,'未找到第三章节点源码，不能把空检查当成通过'
    for source in sources:
        tree = ast.parse(source.read_text(encoding='utf8'))
        for cls in [item for item in tree.body if isinstance(item,ast.ClassDef)]:
            if not any(isinstance(base,ast.Name) and base.id=='Node' for base in cls.bases):
                continue
            for node in ast.walk(cls):
                if (isinstance(node,ast.Attribute) and isinstance(node.ctx,ast.Store) and
                    isinstance(node.value,ast.Name) and node.value.id=='self' and node.attr in readonly):
                    failures.append('%s:%s self.%s'%(source.name,node.lineno,node.attr))
    assert not failures,'Node只读属性冲突：'+', '.join(failures)
