cmake_minimum_required (VERSION 3.5)
project(kimera_vio LANGUAGES CXX C)

find_package(Boost REQUIRED COMPONENTS
    filesystem serialization regex timer date_time thread)

find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)


add_library(${PROJECT_NAME} SHARED "")
add_subdirectory(include/kimera-vio/frontend)
add_subdirectory(include/kimera-vio/backend)
add_subdirectory(include/kimera-vio/imu-frontend)
add_subdirectory(include/kimera-vio/pipeline)
add_subdirectory(include/kimera-vio/dataprovider)
add_subdirectory(src/pipeline)
add_subdirectory(src/imu-frontend)
add_subdirectory(src/frontend)


target_link_libraries(${PROJECT_NAME}

    PUBLIC
    ${OpenCV_LIBRARIES}
    gtsam
    gtsam_unstable
)


target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    # $<INSTALL_INTERFACE:include>
)

add_library(kimera_vio::kimera_vio ALIAS kimera_vio)

add_executable(KimeraVIO ./examples/KimeraVIO.cpp)
target_link_libraries(KimeraVIO PUBLIC kimera_vio::kimera_vio)
