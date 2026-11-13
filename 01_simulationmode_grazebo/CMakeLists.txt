cmake_minimum_required(VERSION 3.8)
project(simulationmode_grazebo)
find_package(ament_cmake REQUIRED)

cmake_policy(SET CMP0009 NEW) 
install(
  DIRECTORY "6-1_sim_env_setup" "6-2_robot_modeling" "6-3_robot_sport"
  DESTINATION share/${PROJECT_NAME}
  PATTERN "_updates" EXCLUDE
  PATTERN ".runtime" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.bak" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)





install(
  FILES 6-3_robot_sport/src/lite3_gait_core.py
  DESTINATION lib/${PROJECT_NAME}
)







# BEGIN LITE3 ONLY
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

ament_package()

