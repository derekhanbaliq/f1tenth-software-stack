cmake_minimum_required(VERSION 3.5)
project(bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  launch/sim_bringup_launch.py
  launch/real_bringup_launch.py
  launch/levine_sim_launch.py
  launch/levine_real_launch.py
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
