cmake_minimum_required (VERSION 3.5)
project(kimera_vio LANGUAGES CXX C)

find_package(Boost REQUIRED COMPONENTS
    filesystem serialization regex timer date_time thread)



add_library(${PROJECT_NAME} SHARED "")
add_subdirectory(include/kimera-vio/frontend)
add_subdirectory(include/kimera-vio/backend)
add_subdirectory(include/kimera-vio/imu-frontend)
add_subdirectory(include/kimera-vio/pipeline)
add_subdirectory(src/pipeline)


target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
)