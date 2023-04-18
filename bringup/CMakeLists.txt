cmake_minimum_required(VERSION 3.5)
project(bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(PROGRAMS
<<<<<<< HEAD
  # launch/sim_bringup_launch.py
  launch/real_bringup_launch.py
=======
  launch/sim_bringup_launch.py
  # launch/real_bringup_launch.py
>>>>>>> c5d33298152a04c4509b8398d9bfeaaaf94f144f
  # DESTINATION lib/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
