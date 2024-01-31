cmake_minimum_required(VERSION 3.16)
project(kitti_motion_compensation)

# some configurationfor general compiler settings #
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS -O3)
set(BUILD_TESTING ON)
add_compile_options(-Wall -Wextra -pedantic -Werror -Wl,--fatal-warnings)

# find packages we need #
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)

# set sources #
include_directories(include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS} 
    )

set(SRC_FILES 
    src/${PROJECT_NAME}/utils.cpp
    src/${PROJECT_NAME}/data_io.cpp
    src/${PROJECT_NAME}/timestamp_mocking.cpp
    src/${PROJECT_NAME}/motion_compensation.cpp
    src/${PROJECT_NAME}/camera_model.cpp
    src/${PROJECT_NAME}/handlers.cpp
    src/${PROJECT_NAME}/lie_algebra.cpp
    src/${PROJECT_NAME}/trajectory_interpolation.cpp
    )

# compile stuff in the core library #
add_library(
    ${PROJECT_NAME}_lib
    SHARED
    ${SRC_FILES}
)

target_link_libraries(
    ${LIBRARY_NAME}
    Eigen3::Eigen
)

# compile examples
set(EXAMPLES 
    generate_projection_visualization_of_run
    motion_compensate_runs
)

foreach(EXAMPLE IN LISTS EXAMPLES)
    add_executable(${EXAMPLE} examples/${EXAMPLE}.cpp)
    target_include_directories(${EXAMPLE} PRIVATE ${INCLUDE_DIRECTORIES})
    # TODO(jack): remove opencv_libs
    target_link_libraries(${EXAMPLE} PRIVATE ${PROJECT_NAME}_lib ${OpenCV_LIBS})
endforeach()

if(BUILD_TESTING)
    enable_testing()
    include(CTest)

    set(TESTS
        test_data_io
        test_motion_compensation
        test_oxts_to_pose 
        test_lie_algebra
        test_timestamp_mocking
        test_trajectory_interpolation
    )

    foreach(TEST IN LISTS TESTS)
        add_executable(${TEST} test/${TEST}.cpp)
        target_include_directories(${TEST} PRIVATE ${GTEST_INCLUDE_DIRS})
        target_link_libraries(${TEST} PRIVATE ${PROJECT_NAME}_lib ${OpenCV_LIBS} ${GTEST_LIBRARIES} pthread)
        add_test(NAME ${TEST} COMMAND ${TEST})
    endforeach()
endif()