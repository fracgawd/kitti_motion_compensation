cmake_minimum_required(VERSION 3.16)
project(kitti_motion_compensation)

# some configurationfor general compiler settings #
set(CMAKE_CXX_STANDARD 17)
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
    src/${PROJECT_NAME}/motion_compensation.cpp
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
    # ${OpenCV_LIBS} # TODO(jack): why can I not do this???
)

# compile some examples #
add_executable(place_holder examples/place_holder.cpp)
target_include_directories(place_holder PRIVATE ${INCLUDE_DIRECTORIES})
# TODO(jack): remove opencv_libs
target_link_libraries(place_holder PRIVATE ${PROJECT_NAME}_lib ${OpenCV_LIBS})

# compile the tests #
add_executable(test_data_io test/test_data_io.cpp)
target_include_directories(test_data_io PRIVATE ${GTEST_INCLUDE_DIRS})
# TODO(jack): remove opencv_libs
target_link_libraries(test_data_io PRIVATE ${PROJECT_NAME}_lib ${GTEST_LIBRARIES} pthread ${OpenCV_LIBS})

add_executable(test_motion_compensation test/test_motion_compensation.cpp)
target_include_directories(test_motion_compensation PRIVATE ${GTEST_INCLUDE_DIRS})
# TODO(jack): remove opencv_libs
target_link_libraries(test_motion_compensation PRIVATE ${PROJECT_NAME}_lib ${GTEST_LIBRARIES} pthread ${OpenCV_LIBS})