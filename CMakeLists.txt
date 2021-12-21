cmake_minimum_required(VERSION 3.8)
project(roscco)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

include(oscc/api/OsccConfig.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(roscco_interfaces REQUIRED)


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


###################################
## catkin specific configuration ##
###################################
ament_package()

###########
## Build ##
###########

include_directories(include oscc/api/include)

set(catkin_EXPORTED_TARGETS rclcpp std_msgs sensor_msgs roscco_interfaces)

add_library(
        oscc_api
        oscc/api/src/oscc.c
)

target_include_directories(
        oscc_api PUBLIC
        include
        oscc/api/include
)

add_library(
        ${PROJECT_NAME}_ros_to_oscc
        src/ros_to_oscc.cpp
)

target_include_directories(
        ${PROJECT_NAME}_ros_to_oscc PUBLIC
        include
        oscc/api/include
)

add_library(
        ${PROJECT_NAME}_oscc_to_ros
        src/oscc_to_ros.cpp
)

target_include_directories(
        ${PROJECT_NAME}_oscc_to_ros PUBLIC
        include
        oscc/api/include
)

target_link_libraries(
        ${PROJECT_NAME}_oscc_to_ros
        oscc_api
)

target_link_libraries(
        ${PROJECT_NAME}_ros_to_oscc
        oscc_api
)

ament_target_dependencies(
        ${PROJECT_NAME}_ros_to_oscc
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

ament_target_dependencies(
        ${PROJECT_NAME}_oscc_to_ros
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

target_include_directories(
        ${PROJECT_NAME}_node PUBLIC
        include
        oscc/api/include
)

ament_target_dependencies(
        ${PROJECT_NAME}_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
        oscc_api
        ${PROJECT_NAME}_ros_to_oscc
        ${PROJECT_NAME}_oscc_to_ros
        )
##TARGET

if (EXAMPLE)
    add_executable(
            ${PROJECT_NAME}_teleop
            example/${PROJECT_NAME}_teleop.cpp
    )

    target_include_directories(
            ${PROJECT_NAME}_teleop PUBLIC
            include
    )

    ament_target_dependencies(
            ${PROJECT_NAME}_teleop
            ${${PROJECT_NAME}_EXPORTED_TARGETS}
            ${catkin_EXPORTED_TARGETS}
    )

    target_link_libraries(
            ${PROJECT_NAME}_teleop
    )

endif ()

if (APOLLO AND (KIA_SOUL_EV OR KIA_NIRO))

    find_package(catkin
            REQUIRED COMPONENTS
            roscpp
            rospy
            std_msgs
            genmsg)

    include_directories(include
            /apollo/bazel-apollo/bazel-out/local-dbg/genfiles/
            )

    add_library(
            ${PROJECT_NAME}_pid_control
            src/pid_control.cpp
    )


    link_directories(
            /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/canbus/
            /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/control/
            /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/common/proto
            /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/drivers/canbus/common/
            /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/drivers/canbus/proto/
            /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/drivers/canbus/)

    add_executable(
            ${PROJECT_NAME}_apollo
            example/${PROJECT_NAME}_apollo.cpp
    )

    target_include_directories(
            ${PROJECT_NAME}_apollo PUBLIC
            include
            oscc/api/include
    )

    ament_target_dependencies(
            ${PROJECT_NAME}_apollo
            ${${PROJECT_NAME}_EXPORTED_TARGETS}
            ${catkin_EXPORTED_TARGETS}
            ${PROJECT_NAME}_apollo_generate_messages_cpp
    )

    target_link_libraries(
            ${PROJECT_NAME}_apollo
            ${PROJECT_NAME}_pid_control
            /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/canbus/proto/libcanbus_proto_lib.a
            /apollo/bazel-apollo/bazel-out/local-dbg/bin/modules/control/proto/libcontrol_proto_lib.a
            canbus_proto_lib
            protobuf
            canbus_lib
            control_lib
            canbus_common
            vehicle_signal_proto_lib
            pnc_point_proto_lib
            gnss_status_proto_lib
            car_sound_proto_lib
            header_proto_lib
            common_proto_lib
            error_code_proto_lib
            vehicle_state_proto_lib
    )

endif ()

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)

    find_package(rostest REQUIRED)

    add_subdirectory(test/rapidcheck)

    add_library(
            oscc_fixture
            test/oscc_fixture.c
    )


    target_include_directories(
            oscc_fixture PUBLIC
            test/include
            oscc/api/include/can_protocols
    )

    add_rostest_gtest(
            test_ros_oscc_api
            test/test_ros_to_oscc.launch
            test/test_ros_to_oscc.cpp
    )

    target_include_directories(
            test_ros_oscc_api PUBLIC
            test/include
            test/rapidcheck/include
            test/rapidcheck/extras/gtest/include
    )

    target_link_libraries(
            test_ros_oscc_api
            rapidcheck
            ${PROJECT_NAME}_ros_to_oscc
            oscc_fixture
    )

    add_rostest_gtest(
            test_oscc_ros_api
            test/test_oscc_to_ros.launch
            test/test_oscc_to_ros.cpp
    )

    target_include_directories(
            test_oscc_ros_api PUBLIC
            include
            test/include
            test/rapidcheck/include
            test/rapidcheck/extras/gtest/include
            oscc/api/include/can_protocols
    )

    target_link_libraries(
            test_oscc_ros_api
            rapidcheck
            ${PROJECT_NAME}_oscc_to_ros
            oscc_fixture
    )

endif ()

#############
## Install ##
#############

install(
        TARGETS
        oscc_api
        ${PROJECT_NAME}_node
        ${PROJECT_NAME}_oscc_to_ros
        ${PROJECT_NAME}_ros_to_oscc
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
        DIRECTORY
        include/${PROJECT_NAME}
        oscc/api/include
        DESTINATION lib/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
)



