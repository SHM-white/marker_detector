cmake_minimum_required(VERSION 3.5)
project(marker_detector)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)

# find dependencies and libraries
set(AMENT_DEPENDENCIES
        rclcpp
        cv_bridge
        std_msgs
        geometry_msgs
        robot_serial
        camera_interfaces
)

foreach (AMENT_DEPENDENCY ${AMENT_DEPENDENCIES})
    find_package(${AMENT_DEPENDENCY} REQUIRED)
endforeach ()

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/DetectResult.msg"
        "msg/DetectResults.msg"
        DEPENDENCIES std_msgs geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)

set(LIBRARIES
        openvino
        ${OpenCV_LIBS}
)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
        src/marker_detector_node.cpp
        include/detector_controller/detector_controller.cpp
        include/detector/detector.cpp
        include/detector/detect_result.cpp
        include/buff_detector/buff_detector.cpp
        include/buff_detector/model_manager/ModelManager.cpp
        include/params/cam_params.cpp
        include/params/buff_params.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})
rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node ${LIBRARIES})
target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/buff_detector/model
        DESTINATION lib/${PROJECT_NAME}/buff_detector/)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
