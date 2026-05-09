cmake_minimum_required(VERSION 3.8)
project(mmwave_radar)
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
 
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
 
install(PROGRAMS
  scripts/radar_module_node.py
  DESTINATION lib/${PROJECT_NAME}
)
 
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
 
ament_package()