cmake_minimum_required(VERSION 3.5)
project(raspi_camera_provider)


set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake/")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-as-needed")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH "/usr/local/lib/cmake/${CMAKE_MODULE_PATH}") 

# find dependencies
# find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(raspicam REQUIRED)

find_package(PkgConfig)
#pkg_check_modules(SERIAL libserial)

add_executable(raspi_p src/utilities/base64/base64.cpp src/nodetemplate.cpp src/settings.cpp src/lifecycle_node.cpp src/metadata/metadata_manager.cpp src/state_machine/lifecycle_state_manager.cpp src/node_network/node_network_manager.cpp src/state_machine/lifecycle_state.cpp src/state_machine/states/unconfigured_state.cpp src/state_machine/states/inactive_state.cpp src/state_machine/states/active_state.cpp)
ament_target_dependencies(raspi_p rclcpp std_msgs std_srvs)
target_include_directories(raspi_p PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# added libraries
# target_link_libraries(raspi_p ${OpenCV_LIBS})
target_link_libraries(raspi_p ${raspicam_LIBS})

install(TARGETS raspi_p
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
