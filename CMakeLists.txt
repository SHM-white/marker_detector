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

# 目前支持的识别方法：MobileNet, BP, Template
set(RECOGNITION_METHOD "BP")
# 目前支持的后端引擎：openvino, tensorrt, 也可以设置为auto或none
set(NN_BACKEND "openvino")

add_subdirectory(include/armor_detector/DigitalRecognition)

include(include/armor_detector/DigitalRecognition/path.cmake)
include(include/armor_detector/DigitalRecognition/func.cmake)

detectBackend()
detectMethod()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/DetectResult.msg"
        "msg/DetectResults.msg"
        DEPENDENCIES std_msgs geometry_msgs
)

set(LIBRARIES
        openvino
        ${CERES_LIBRARIES}
        DigitalRecognitionLib
)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

ament_auto_add_executable(${PROJECT_NAME}_node
        src/marker_detector_node.cpp
        src/detector_controller/detector_controller.cpp
        src/detector/detector.cpp
        src/detector/detect_result.cpp
        src/buff_detector/buff_detector.cpp
        include/buff_detector/model_manager/ModelManager.cpp
        src/params/cam_params.cpp
        src/params/buff_params.cpp
        src/armor_detector/armor_detector.cpp
        src/armor_detector/Marker.cpp
        src/armor_detector/MarkSensor.cpp
        src/armor_detector/RotRect.cpp
        src/armor_detector/utils.cpp
        src/armor_detector/history_manager.cpp
        src/params/armor_params.cpp
)

rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node ${LIBRARIES})
target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

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

ament_auto_package()

