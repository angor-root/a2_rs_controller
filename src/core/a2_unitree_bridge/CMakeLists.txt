cmake_minimum_required(VERSION 3.5)
project(a2_unitree_bridge)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(a2_interfaces REQUIRED)
find_package(a2_utils REQUIRED)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Shared RPATH helper — keeps the Unitree SDK libs resolvable at runtime.
function(apply_unitree_rpath target)
  set_target_properties(${target} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN:/opt/unitree_robotics/lib"
  )
  # RPATH instead of RUNPATH so Unitree libs load before LD_LIBRARY_PATH.
  target_link_options(${target} PRIVATE "-Wl,--disable-new-dtags")
  target_link_libraries(${target} pthread unitree_sdk2)
endfunction()

# ── Robot bridge ──────────────────────────────────────────────────────────────
add_executable(a2_bridge_robot
  src/node.cpp
  src/bridge.cpp
  src/robot/command_egress.cpp
)
ament_target_dependencies(a2_bridge_robot
  rclcpp ament_index_cpp unitree_hg unitree_go std_msgs sensor_msgs a2_interfaces geometry_msgs
  a2_utils
)
target_link_libraries(a2_bridge_robot ${a2_utils_LIBRARIES})
apply_unitree_rpath(a2_bridge_robot)

# ── Sim bridge ────────────────────────────────────────────────────────────────
add_executable(a2_bridge_sim
  src/node.cpp
  src/bridge.cpp
)
target_compile_definitions(a2_bridge_sim PRIVATE A2_MODE_SIM)
ament_target_dependencies(a2_bridge_sim
  rclcpp ament_index_cpp unitree_go std_msgs sensor_msgs camera_info_manager
  nav_msgs geometry_msgs tf2_ros tf2_sensor_msgs tf2 a2_interfaces
)
apply_unitree_rpath(a2_bridge_sim)

install(TARGETS a2_bridge_robot a2_bridge_sim
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
