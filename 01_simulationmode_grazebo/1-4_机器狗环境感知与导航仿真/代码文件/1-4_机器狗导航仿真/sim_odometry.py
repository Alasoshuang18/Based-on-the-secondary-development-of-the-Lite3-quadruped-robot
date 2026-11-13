#!/usr/bin/env python3
"""Gazebo truth-assisted odometry for this simulation's 2-D SLAM exercise.

This node DOES NOT estimate odometry from lidar, IMU, joints, or commands.
It reads /lite3_sim/model_pose and exposes the simulator's model pose as
/lite3_sim/odom and odom -> base_footprint -> TORSO. This is a deliberately
labelled simulation aid, not a replacement for real-robot odometry and not
evidence that a lidar-inertial estimator has been validated. SLAM alone owns
map -> odom. The original source timestamps are preserved without restamping.
"""
import math
import signal


def _multiply(left, right):
    """Hamilton quaternion product, ROS xyzw order."""
    ax, ay, az, aw = left
    bx, by, bz, bw = right
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def split_pose(translation, quaternion):
    """Split a finite world pose into planar base and residual torso rotation.

    Returns (xyz, normalized_full_q, yaw, yaw_q, residual_q). Quaternion
    multiplication yaw_q * residual_q reconstructs the original orientation,
    including roll and pitch; torso z is never replaced with a guessed height.
    """
    if len(translation) != 3 or len(quaternion) != 4:
        raise ValueError('Expected xyz and ROS xyzw quaternion')
    xyz = tuple(float(value) for value in translation)
    q = tuple(float(value) for value in quaternion)
    if not all(math.isfinite(value) for value in xyz + q):
        raise ValueError('Non-finite model pose')
    norm = math.hypot(*q)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid model quaternion')
    q = tuple(value / norm for value in q)
    x, y, z, w = q
    yaw = math.atan2(2.*(w*z+x*y), 1.-2.*(y*y+z*z))
    yaw_q = (0., 0., math.sin(yaw/2.), math.cos(yaw/2.))
    residual_q = _multiply((0., 0., -yaw_q[2], yaw_q[3]), q)
    return xyz, q, yaw, yaw_q, residual_q


def main(args=None):
    # ROS imports stay here so pose decomposition can be independently checked.
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    from tf2_ros import TransformBroadcaster

    class GazeboTruthOdometry(Node):
        def __init__(self):
            super().__init__('lite3_sim_ground_truth_odometry',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.publisher = self.create_publisher(Odometry, '/lite3_sim/odom', 10)
            self.broadcaster = TransformBroadcaster(self)
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.subscription = self.create_subscription(
                TFMessage, '/lite3_sim/model_pose', self.receive, qos)
            self.previous = None
            self.rejected = 0
            self.get_logger().info(
                'SIMULATION GROUND TRUTH AID: model_pose -> /lite3_sim/odom; '
                'TF odom -> base_footprint -> TORSO. No map TF, no control '
                'commands, and no lidar/IMU odometry estimation.')

        def reject(self, reason):
            self.rejected += 1
            if self.rejected == 1 or self.rejected % 100 == 0:
                self.get_logger().warning('Rejected model pose: '+reason)

        @staticmethod
        def rotation(message, values):
            message.x, message.y, message.z, message.w = values

        def receive(self, message):
            for source in message.transforms:
                if source.child_frame_id != 'lite3_control':
                    continue
                if source.header.frame_id != 'lite3_world':
                    self.reject('expected parent lite3_world')
                    continue
                stamp = source.header.stamp
                if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
                    self.reject('invalid source timestamp')
                    continue
                stamp_ns = stamp.sec*1_000_000_000 + stamp.nanosec
                t, q = source.transform.translation, source.transform.rotation
                try:
                    xyz, _, yaw, yaw_q, residual = split_pose(
                        (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
                except ValueError as exc:
                    self.reject(str(exc))
                    continue
                previous = self.previous
                if previous is not None and stamp_ns <= previous[0]:
                    if stamp_ns < previous[0] - 1_000_000_000:
                        # A reset needs a fresh SLAM session. Keep this truth
                        # bridge useful, but never silently pretend map history
                        # can continue through a world reset.
                        self.get_logger().warning(
                            'Simulation/model timestamp reset: restart the SLAM '
                            'session before continuing an existing map.')
                        previous = None
                        self.previous = None
                    else:
                        continue
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_footprint'
                odom.pose.pose.position.x = xyz[0]
                odom.pose.pose.position.y = xyz[1]
                self.rotation(odom.pose.pose.orientation, yaw_q)
                # Velocity is a finite difference of simulator truth, expressed
                # in the planar child frame. It is not leg/IMU state estimation.
                if previous is not None:
                    dt = (stamp_ns-previous[0])/1e9
                    if 0. < dt <= .5:
                        vx = (xyz[0]-previous[1])/dt
                        vy = (xyz[1]-previous[2])/dt
                        c, s = math.cos(yaw), math.sin(yaw)
                        odom.twist.twist.linear.x = c*vx+s*vy
                        odom.twist.twist.linear.y = -s*vx+c*vy
                        dyaw = (yaw-previous[3]+math.pi) % (2.*math.pi)-math.pi
                        odom.twist.twist.angular.z = dyaw/dt
                base = TransformStamped()
                base.header.stamp = stamp
                base.header.frame_id = 'odom'
                base.child_frame_id = 'base_footprint'
                base.transform.translation.x = xyz[0]
                base.transform.translation.y = xyz[1]
                self.rotation(base.transform.rotation, yaw_q)
                torso = TransformStamped()
                torso.header.stamp = stamp
                torso.header.frame_id = 'base_footprint'
                torso.child_frame_id = 'TORSO'
                torso.transform.translation.z = xyz[2]
                self.rotation(torso.transform.rotation, residual)
                self.publisher.publish(odom)
                self.broadcaster.sendTransform([base, torso])
                self.previous = (stamp_ns, xyz[0], xyz[1], yaw)
                # Exactly one canonical model pose is accepted per TFMessage.
                break

    rclpy.init(args=args)
    node = GazeboTruthOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
