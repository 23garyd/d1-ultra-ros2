cmake_minimum_required(VERSION 3.8)
project(robots_dog_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Localization.msg"
  "msg/LocalizationState.msg"
  "msg/NavigationError.msg"
  "msg/NavigationErrorReport.msg"
  "msg/NavigationState.msg"
  "msg/SlamState.msg"
  "msg/StartNavigation.msg"
  "srv/GetMapState.srv"
  "srv/IsExplorationGoalValid.srv"
  "srv/IsExplorationGoalsValid.srv"
  "srv/LoadMap.srv"
  "srv/MapState.srv"
  "srv/SetLocalizationType.srv"
  DEPENDENCIES builtin_interfaces diagnostic_msgs geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # test_contract.py (generator-tool test) is omitted from this copy: it requires
  # tools/extract_d1_interfaces.py from the tracer-mini-d1-fastlio source repo.
  # Wire identity is guaranteed by test_generated_interfaces against contract/ dumps.
  ament_add_pytest_test(test_generated_interfaces test/test_generated_interfaces.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
