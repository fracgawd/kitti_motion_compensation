cmake_minimum_required(VERSION 3.16)
project(kitti_motion_compensation)

# some configurationfor general compiler settings@S
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -pedantic -Werror -Wl,--fatal-warnings)

# find packages we need 
find_package(Eigen3 REQUIRED)

# set sources
include_directories(include
    ${EIGEN3_INCLUDE_DIR}
    )

set(SRC_FILES 
    src/${PROJECT_NAME}/data_io.cpp
    )

# compile stuff the core library
add_library(
    ${PROJECT_NAME}_lib
    SHARED
    ${SRC_FILES}
)

target_link_libraries(
    ${LIBRARY_NAME}
    Eigen3::Eigen
)

# compile some examples
add_executable(place_holder examples/place_holder.cpp)
target_include_directories(place_holder PRIVATE ${INCLUDE_DIRECTORIES})
target_link_libraries(place_holder PRIVATE ${PROJECT_NAME}_lib)

# compile the tests