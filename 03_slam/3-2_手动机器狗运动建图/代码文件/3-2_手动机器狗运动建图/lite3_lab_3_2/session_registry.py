"""Read-only session directory endpoint for the unchanged prepare_nav.py."""
from pathlib import Path
import rclpy
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node=Node('lab_4_2')
    node.declare_parameter('result_dir','')
    directory=Path(node.get_parameter('result_dir').value)
    if not directory.is_absolute() or not directory.is_dir():
        raise ValueError('A valid mapping result directory is required')
    node.get_logger().info('本轮目录登记：'+str(directory)+'；保存服务为/course/save_map。')
    from .timing_record import TimingRecord
    timing=TimingRecord(node,directory)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        timing.close()
        node.destroy_node()
        rclpy.try_shutdown()
