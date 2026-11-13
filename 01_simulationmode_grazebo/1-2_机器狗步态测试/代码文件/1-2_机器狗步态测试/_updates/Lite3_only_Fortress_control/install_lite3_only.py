"""Install only inside the user's existing simulationmode_grazebo directory."""
import ast
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil
import uuid
import xml.etree.ElementTree as ET


def contained(path, root):
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError('Path outside the allowed package: '+str(path))


def install(update, home):
    package = home/'catkin_ws_lite3dog/src/06_simulationmode_grazebo'
    update = update.resolve()
    chapter = package/'6-3_robot_sport'
    allowed_uploads = (package/'Lite3_only_Fortress_control',
                      chapter/'_updates/Lite3_only_Fortress_control')
    if update not in tuple(path.resolve() for path in allowed_uploads):
        raise ValueError('请把更新包上传到 '+str(chapter/'_updates'))
    if ET.parse(package/'package.xml').getroot().findtext('name') != 'simulationmode_grazebo':
        raise ValueError('目标 ROS 包名不匹配')
    model = package/'6-2_robot_modeling/urdf/Lite3.urdf'
    if not model.is_file():
        raise ValueError('缺少现有 Lite3 模型：'+str(model))
    cmake_path = package/'CMakeLists.txt'
    original = cmake_path.read_text(encoding='utf-8')
    if 'ament_package()' not in original:
        raise ValueError('目标 CMakeLists.txt 缺少 ament_package()')
    manifest = json.loads((update/'MANIFEST_SHA256.json').read_text(encoding='utf-8'))
    for relative, digest in manifest['files'].items():
        path = update/relative
        contained(path, update)
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('更新文件缺失或校验失败：'+relative)
        if path.suffix == '.py':
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
        if relative.split('/')[0] in ('src','launch','config','worlds','env'):
            contained(package/'6-3_robot_sport'/relative, package)
    for relative in ('package.xml','CMakeLists.txt','6-3_robot_sport',
                     '.lite3_backups','.lite3_runtime','lite3_env.bash','lite3_fortress_control',
                     '6-3_robot_sport/.runtime','6-3_robot_sport/_updates','6-3_robot_sport/tests'):
        contained(package/relative, package)
    legacy = ('src/check_foot_lift.py','src/gait_control.py','src/keyboard_control.py',
              'src/measured_gait_control.py','src/__pycache__',
              'TESTING_V4_1.md','TESTING_V5.md',
              'launch/control_spawn.launch.py','launch/foot_visual_spawn.launch.py',
              'launch/gait.launch.py','launch/walk_spawn.launch.py',
              'launch/pose_debug_bridge.launch.py','launch/upstream_spawn.launch.py',
              'config/controllers.yaml','config/gait.yaml','config/upstream_controllers.yaml',
              'worlds/lite3_upstream.sdf')
    for relative in legacy:
        contained(chapter/relative, package)
    # Remove explicit install rules for the retired chapter scripts as well as
    # our earlier generated blocks. Never drop a mixed rule containing course
    # sections 6-1 or 6-2 without inspecting the real CMake source.
    for marker in ('LITE3 UPSTREAM FORTRESS','LITE3 V5','LITE3 ONLY'):
        original = re.sub(r'(?ms)^# BEGIN '+re.escape(marker)+r'\n.*?^# END '+re.escape(marker)+r'\n?', '', original)
    def retire_install(match):
        statement = match.group(0)
        if any('6-3_robot_sport/'+path in statement for path in legacy):
            if '6-1_sim_env_setup' in statement or '6-2_robot_modeling' in statement:
                raise ValueError('CMake 中旧章节入口与 6-1/6-2 混在同一 install 规则；请先检查该规则')
            return ''
        return statement
    original = re.sub(r'(?is)\binstall\s*\((?:[^()]|\([^()]*\))*\)', retire_install, original)

    backup_base = package/'.lite3_backups'
    backup_base.mkdir(exist_ok=True)
    (backup_base/'COLCON_IGNORE').touch()
    backup = backup_base/('update_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    backup.mkdir()
    (backup/'COLCON_IGNORE').touch()
    for relative in ('package.xml','CMakeLists.txt','lite3_env.bash'):
        if (package/relative).is_file():
            shutil.copy2(package/relative, backup/relative)
    if chapter.is_dir():
        shutil.copytree(chapter, backup/'6-3_robot_sport',
                        ignore=shutil.ignore_patterns('_updates','.runtime','__pycache__'))
    # Keep all archived versions recoverable; do not remove unknown source/ data.
    for relative in legacy:
        old = chapter/relative
        if old.exists():
            archived = backup/'retired'/relative
            archived.parent.mkdir(parents=True,exist_ok=True)
            shutil.move(str(old),str(archived))
    if (chapter/'tests').is_dir():
        shutil.move(str(chapter/'tests'),str(backup/'retired/tests'))
    old_cpp = package/'lite3_fortress_control'
    if old_cpp.is_dir():
        if (old_cpp/'COLCON_IGNORE').is_file():
            shutil.copy2(old_cpp/'COLCON_IGNORE', backup/'old_cpp_COLCON_IGNORE')
        (old_cpp/'COLCON_IGNORE').touch()
        shutil.move(str(old_cpp),str(backup/'retired_cpp_sources'))
    (update/'COLCON_IGNORE').touch()
    if (package/'.lite3_runtime').is_dir():
        shutil.move(str(package/'.lite3_runtime'),str(backup/'old_package_runtime'))
    runtime = chapter/'.runtime'
    runtime.mkdir(exist_ok=True)
    (runtime/'COLCON_IGNORE').touch()
    for relative in manifest['files']:
        if relative.split('/')[0] in ('src','launch','config','worlds','env','tests'):
            destination = chapter/relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(update/relative, destination)
    shutil.copy2(update/'README.md', chapter/'LITE3_ONLY_TESTING.md')

    root = ET.parse(package/'package.xml').getroot()
    for node in list(root):
        if node.tag.endswith('depend') and (node.text or '').strip() in (
                'unitree_guide_controller','leg_pd_controller','controller_common'):
            root.remove(node)
    dependencies = ('rclpy','ament_index_python','launch','launch_ros','robot_state_publisher',
        'ros_gz_sim','ros_gz_bridge','gz_ros2_control','controller_manager','controller_manager_msgs',
        'joint_state_broadcaster','joint_trajectory_controller','control_msgs','trajectory_msgs',
        'geometry_msgs','sensor_msgs','tf2_msgs','std_msgs','std_srvs')
    present = {n.text.strip() for n in root if n.tag in ('depend','exec_depend') and n.text}
    for name in dependencies:
        if name not in present:
            ET.SubElement(root,'exec_depend').text = name
    ET.indent(root, space='  ')
    (package/'package.xml').write_text('<?xml version="1.0"?>\n'+ET.tostring(root,encoding='unicode')+'\n',encoding='utf-8')
    for marker in ('LITE3 UPSTREAM FORTRESS','LITE3 V5','LITE3 ONLY'):
        original = re.sub(r'(?ms)^# BEGIN '+re.escape(marker)+r'\n.*?^# END '+re.escape(marker)+r'\n?', '', original)
    block = '''# BEGIN LITE3 ONLY
install(FILES "6-3_robot_sport/launch/lite3_spawn.launch.py" "6-3_robot_sport/launch/lite3_gait.launch.py"
  DESTINATION share/${PROJECT_NAME}/6-3_robot_sport/launch)
install(FILES "6-3_robot_sport/config/lite3_controllers.yaml" "6-3_robot_sport/config/lite3_gait.yaml"
  DESTINATION share/${PROJECT_NAME}/6-3_robot_sport/config)
install(FILES "6-3_robot_sport/worlds/lite3_walk.sdf"
  DESTINATION share/${PROJECT_NAME}/6-3_robot_sport/worlds)
install(PROGRAMS "6-3_robot_sport/src/lite3_controller.py"
  DESTINATION lib/${PROJECT_NAME} RENAME lite3_controller)
install(PROGRAMS "6-3_robot_sport/src/lite3_keyboard.py"
  DESTINATION lib/${PROJECT_NAME} RENAME lite3_only_keyboard)
install(FILES "6-3_robot_sport/src/lite3_feedback_plumbing.py"
  "6-3_robot_sport/src/lite3_gait_core.py" "6-3_robot_sport/src/lite3_measured_core.py"
  "6-3_robot_sport/src/lite3_feedback_sync.py" "6-3_robot_sport/src/foot_debug_core.py"
  DESTINATION lib/${PROJECT_NAME})
# END LITE3 ONLY
'''
    cmake_path.write_text(original.replace('ament_package()',block+'\nament_package()',1),encoding='utf-8')
    environment = '''# Compatibility entry; the environment implementation is in chapter 6-3.
source "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/env/lite3_env.bash"
'''
    (package/'lite3_env.bash').write_text(environment,encoding='utf-8')
    print('安装完成：只更新 '+str(package))
    print('备份：'+str(backup))
    old_upload = package/'Lite3_only_Fortress_control'
    if update != old_upload.resolve() and old_upload.is_dir():
        contained(old_upload,package)
        shutil.move(str(old_upload),str(backup/'old_upload'))
    print('下一步：在本包构建到 6-3_robot_sport/.runtime，再加载该章节 env/lite3_env.bash。')
    return backup


if __name__ == '__main__':
    install(Path(__file__).resolve().parent, Path.home())
