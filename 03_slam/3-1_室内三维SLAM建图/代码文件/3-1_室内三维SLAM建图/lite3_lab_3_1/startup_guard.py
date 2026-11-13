"""Reject overlapping real LIO sessions without killing any process."""
import os
from pathlib import Path


COURSE_NODES = {'sensor_adapter', 'lio_starter', 'map_archive', 'fastlio_mapping'}


def inspect_processes(proc_root=Path('/proc'), uid=None):
    uid = os.getuid() if uid is None else uid
    result = []
    for folder in proc_root.iterdir():
        if not folder.name.isdigit():
            continue
        try:
            status = dict(line.split(':', 1) for line in (folder/'status').read_text().splitlines() if ':' in line)
            if int(status['Uid'].split()[0]) != uid or status.get('State', '').lstrip().startswith('Z'):
                continue
            argv = (folder/'cmdline').read_bytes().decode().strip('\0').split('\0')
            env = dict(item.split('=', 1) for item in (folder/'environ').read_bytes().decode().split('\0') if '=' in item)
            result.append({'pid': int(folder.name), 'argv': argv,
                           'domain': int(env.get('ROS_DOMAIN_ID', '0'))})
        except (OSError, UnicodeError, ValueError, KeyError):
            continue
    return result


def check_real_runtime(processes, domain, start_hardware, workspace):
    """Only inspect same-user local processes; no signals, sockets or robot commands."""
    workspace = str(Path(workspace)) + '/'
    conflicts, drivers = [], []
    for process in processes:
        argv = process['argv']
        # Python entry points occupy argv[1], native executables argv[0].
        names = {Path(arg).name for arg in argv[:2] if arg}
        command = ' '.join(argv)
        if 'livox_ros_driver2_node' in names:
            drivers.append(process)
        if process['domain'] != domain:
            continue
        is_course_node = bool(names & COURSE_NODES)
        if is_course_node and (workspace in command or 'fastlio_mapping' in names):
            conflicts.append(process)
        if 'native_runner' in names and workspace in command:
            conflicts.append(process)
        if any(Path(arg).name == 'ros2' and argv[index+1:index+3] == ['bag', 'play']
               for index, arg in enumerate(argv[:2])):
            conflicts.append(process)
    if conflicts:
        details = '; '.join('PID=%d %s' % (p['pid'], ' '.join(p['argv'])) for p in conflicts)
        raise RuntimeError(
            '4-1启动前检查：发现同域已有LIO/适配器/回放或原生执行进程，拒绝叠加启动。'
            '即使旧父launch已被强制结束，子进程也可能仍存活。先确认旧会话归属，'
            '有运动执行时先正常交还，再在原终端Ctrl+C退出；不要kill -9、删除锁文件或直接复用残留。'
            + details)
    same_domain = [p for p in drivers if p['domain'] == domain]
    other_domain = [p for p in drivers if p['domain'] != domain]
    if start_hardware and drivers:
        raise RuntimeError('4-1启动前检查：已有Livox驱动进程，不再启动第二份。仅复用已核对的单一同域驱动，'
                           '或正常退出其原launch后再启动。PIDs=' + ','.join(str(p['pid']) for p in drivers))
    if not start_hardware and (len(same_domain) != 1 or other_domain):
        raise RuntimeError('4-1启动前检查：start_hardware=false仅允许恰好一个已核对的同域Livox驱动；'
                           '当前同域=%d，其他域=%d。不要仅凭pgrep有输出就复用。' % (len(same_domain), len(other_domain)))


def preflight(context):
    from launch.substitutions import LaunchConfiguration
    def value(name):
        return LaunchConfiguration(name).perform(context)
    if value('mode') != 'real' or value('start_runtime').lower() != 'true':
        return []
    check_real_runtime(inspect_processes(), int(os.environ.get('ROS_DOMAIN_ID', '0')),
                       value('start_hardware').lower() == 'true',
                       Path.home()/'catkin_ws_lite3dog')
    return []
