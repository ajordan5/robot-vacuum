cmake_minimum_required(VERSION 3.14)

project(RobotVacuumSim LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 COMPONENTS Widgets Concurrent Gui 3DRender 3DInput 3DExtras 3DCore REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED )

add_subdirectory(occupancygrid/)
add_subdirectory(simulation/)

option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    find_package(GTest REQUIRED)

    add_executable(occupancygrid_unittests)

    target_sources(occupancygrid_unittests PRIVATE tests/occupancygrid_unittests.cpp)
    target_include_directories(occupancygrid_unittests PRIVATE "${GTEST_INCLUDE_DIRS}")
    target_link_libraries(occupancygrid_unittests
        PRIVATE
            ${GTEST_LIBRARIES}
            ${GTEST_MAIN_LIBRARIES}
            RobotVacuumSim
            OMap
    )
endif(BUILD_TESTS)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
add_subdirectory(gui/)


