cmake_minimum_required(VERSION 3.14)

project(RobotVacuumSim LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Bullet REQUIRED )
find_package(Qt6 COMPONENTS Widgets Concurrent Gui 3DRender 3DInput 3DExtras 3DCore REQUIRED)

find_package(Eigen3 REQUIRED)

add_library( OMap STATIC
  occupancygrid.cpp
  occupancygrid.h
  occupancygridutils.cpp
  occupancygridutils.h
  lidarmapper.h
  lidarmapper.cpp

)

target_link_libraries(OMap Eigen3::Eigen)

add_library(RobotVacuumSim STATIC
  vacuum.cpp
  vacuum.h
  ground.cpp
  ground.h
  obstacles.h
  obstacles.cpp
  lidar.h
  lidar.cpp
  world.h
  world.cpp
  simutils.cpp
  simutils.h

)

target_link_directories(RobotVacuumSim PRIVATE ${BULLET_LIBRARY_DIRS})
target_compile_definitions(RobotVacuumSim PRIVATE ROBOTVACUUMSIM_LIBRARY ${BULLET_DEFINITIONS})
target_include_directories(RobotVacuumSim  PUBLIC ${BULLET_INCLUDE_DIRS} ${BULLET_PHYSICS_SOURCE_DIR}/examples/ExampleBrowser)
target_link_libraries(RobotVacuumSim
    ${BULLET_LIBRARIES}
    Eigen3::Eigen
    Qt6::Widgets
    Qt6::Gui
    Qt6::3DRender
    Qt6::3DCore
    Qt6::3DInput
    Qt6::3DExtras
    OMap)


option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    find_package(GTest REQUIRED)

    add_executable(occupancygrid_unittests)

    target_sources(occupancygrid_unittests PRIVATE occupancygrid_unittests.cpp)
    target_include_directories(occupancygrid_unittests PRIVATE "${GTEST_INCLUDE_DIRS}")
    target_link_libraries(occupancygrid_unittests
        PRIVATE
            ${GTEST_LIBRARIES}
            ${GTEST_MAIN_LIBRARIES}
            RobotVacuumSim
            OMap
    )
endif(BUILD_TESTS)

add_executable(App_RobotVacuum
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mappainter.h
    mappainter.cpp
    mainwindow.ui
)

target_compile_definitions(App_RobotVacuum PRIVATE ${BULLET_DEFINITIONS})
target_include_directories(App_RobotVacuum
    PRIVATE
    ${BULLET_INCLUDE_DIRS}
)

target_link_libraries(App_RobotVacuum
    Qt6::Widgets
    Qt6::Gui
    Qt6::3DRender
    Qt6::3DCore
    Qt6::3DInput
    Qt6::3DExtras
    ${BULLET_LIBRARIES}
    RobotVacuumSim
    )

