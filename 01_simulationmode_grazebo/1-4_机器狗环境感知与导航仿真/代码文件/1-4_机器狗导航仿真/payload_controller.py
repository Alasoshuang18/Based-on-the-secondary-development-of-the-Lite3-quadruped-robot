#!/usr/bin/env python3
"""Reuse crawl or opt-in diagonal trot with the actual 6-4 mounted mass model.

The existing package and all 6-3 files are read-only. A canonical description
fingerprint prevents a stale generated URDF from silently selecting wrong COM.
"""
import hashlib
import importlib.machinery
import importlib.util
from pathlib import Path
import signal
import sys
import xml.etree.ElementTree as ET

# Reused 6-3 modules are read-only, including their Python cache directories.
sys.dont_write_bytecode = True

import rclpy
from ament_index_python.packages import get_package_prefix
from payload_trot_diagnostics import TrotTrace
from payload_speed_trial import controller_tree, planner_class
from payload_stance_control import stance_class

ROOT = Path(__file__).resolve().parent
LIB = Path(get_package_prefix('simulationmode_grazebo'))/'lib/simulationmode_grazebo'
sys.path.insert(0,str(LIB))
loader = importlib.machinery.SourceFileLoader('navigation_reused_gait',str(LIB/'lite3_controller'))
spec = importlib.util.spec_from_loader(loader.name,loader)
base = importlib.util.module_from_spec(spec)
# Compile a checked, process-local speed envelope; never write into LIB.
source_path = LIB/'lite3_controller'
exec(compile(controller_tree(source_path.read_text(), str(source_path)),
             str(source_path), 'exec'), base.__dict__)


def fingerprint(xml):
    return hashlib.sha256(ET.canonicalize(xml,strip_text=True).encode()).hexdigest()


class PayloadGait(base.Gait):
    def __init__(self):
        self.trot_trace = TrotTrace()
        super().__init__()
        base.DiagonalTrot = stance_class(base.DiagonalTrot)
        speed_trial = self.declare_parameter('nav_enable_speed_trial', False).value
        if not isinstance(speed_trial, bool):
            raise ValueError('nav_enable_speed_trial must be boolean')
        if self.trot_speed > .020:
            if not speed_trial or self.gait_mode != 'trot':
                raise ValueError('Above 0.020 m/s requires trot and nav_enable_speed_trial:=true')
            self.get_logger().warning(f'NAV_SPEED_TRIAL: candidate cap={self.trot_speed:.3f} m/s; '
                                      'joint/contact/feedback protections retained; manual test only')
        base.DiagonalTrot = planner_class(base.DiagonalTrot, self.trot_speed)
        if self.pin_body:
            raise ValueError('Payload entry requires pin_body=false')
        path = ROOT/'.runtime/lite3_navigation.urdf'
        xml = path.read_text(encoding='utf-8')
        root = ET.fromstring(xml)
        if root.find("link[@name='mid360_link']") is None:
            raise ValueError('Generated navigation URDF lacks mounted lidar')
        self.expected_description = fingerprint(xml)
        self.mass_model = base.URDFCenterOfMass(path)
        self.get_logger().info(f'NAVIGATION_PAYLOAD: mode={self.gait_mode}; reused={base.VERSION}; total_mass_kg={self.mass_model.total_mass:.6f}; '
                               f'COM from {path}; original 6-3 files unchanged')
        if self.gait_mode == 'trot':
            self.get_logger().info('NAV_STANCE_FEEDBACK: stance IK XY correction <=10 mm from measured body; '
                                   'COM recenter gain=3/s with reference anti-windup, transfer <=20 mm/s; '
                                   'world anchors and 40 mm absolute-reference watchdog unchanged')
            self.get_logger().info('Four-foot qualification still tests HL,FL,HR,FR individually; '
                                   'subsequent walking uses simultaneous FL+HR / FR+HL swings')
            self.get_logger().info('TROT_TRACE: retains last 300 planner snapshots; '
                                   f'20 mm support-offset guard; speed cap={self.trot_speed:.3f} m/s')

    def run_trot(self, dt):
        try:
            return super().run_trot(dt)
        finally:
            # Capture the failing update before the original FAULT path clears
            # planner state. Diagnostic errors must never replace a gait error.
            try:
                self.trot_trace.append(self)
                if (self.state == 'READY' and self.trot is not None and self.trot.done
                        and not getattr(self, 'trot_trace_saved', False)):
                    path = self.trot_trace.save(ROOT/'.runtime/diagnostics', 'Stopped after verified touchdown')
                    self.trot_trace_saved = True
                    self.get_logger().info('Trot completed-run evidence: '+str(path))
            except Exception as exc:
                self.get_logger().warning('Trot snapshot unavailable: '+str(exc))

    def begin_trot(self):
        self.trot_trace = TrotTrace()
        self.trot_trace_saved = False
        return super().begin_trot()

    def fault(self, reason):
        # The original stop/protection runs even if evidence writing fails.
        try:
            if getattr(self, 'trot', None) is not None and self.state != 'FAULT':
                path = self.trot_trace.save_fault(ROOT/'.runtime/diagnostics', reason)
                self.get_logger().error('Trot fault evidence: '+str(path))
        except Exception as exc:
            self.get_logger().warning('Trot fault evidence unavailable: '+str(exc))
        finally:
            super().fault(reason)

    def readiness(self):
        if self.count_publishers('/lite3_sim/lite3_leg_controller/joint_trajectory') != 1:
            return 'Another joint-command publisher exists; stop the other gait terminal first'
        return super().readiness()

    def model_description(self,message):
        super().model_description(message)
        try:
            if fingerprint(message.data) != self.expected_description:
                self.model_mode_error = 'Mounted robot_description differs from COM model; restart C/E consistently'
        except (ET.ParseError,ValueError):
            self.model_mode_error = 'Invalid mounted robot_description'


def main():
    rclpy.init()
    node = None
    try:
        node = PayloadGait()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # A subscription take can fail after the ROS context is shut down.
        # Preserve every exception while the controller context is still live.
        if rclpy.ok():
            raise
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node is not None: node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__': main()
