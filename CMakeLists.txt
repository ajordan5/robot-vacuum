cmake_minimum_required(VERSION 3.14)

project(RobotVacuumSim LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "C:/bullet3/lib")
list(APPEND CMAKE_PREFIX_PATH "C:/bullet3/lib/Release")
#list(APPEND CMAKE_PREFIX_PATH "C:/bullet3/build3/vs2010")
list(APPEND CMAKE_PREFIX_PATH "C:/bullet3/src")
list(APPEND CMAKE_PREFIX_PATH "C:/bullet3/examples")
list(APPEND CMAKE_PREFIX_PATH "C:/eigen-3.4.0/cmake")
list(APPEND CMAKE_PREFIX_PATH "C:/")
find_package(Bullet REQUIRED )

add_library(RobotVacuumSim STATIC
  robotvacuumsim.cpp
  robotvacuumsim.h

)

target_link_directories(RobotVacuumSim PRIVATE ${BULLET_LIBRARY_DIRS})
target_compile_definitions(RobotVacuumSim PRIVATE ROBOTVACUUMSIM_LIBRARY ${BULLET_DEFINITIONS})
target_include_directories(RobotVacuumSim  PUBLIC ${BULLET_INCLUDE_DIRS} ${BULLET_PHYSICS_SOURCE_DIR}/examples/ExampleBrowser)
target_link_libraries(RobotVacuumSim ${BULLET_LIBRARIES})

find_package(Eigen3 REQUIRED)

add_library( OMap STATIC
  occupancygrid.cpp
  occupancygrid.h
  occupancygridutils.cpp
  occupancygridutils.h

)

target_link_libraries(OMap Eigen3::Eigen)

list(APPEND CMAKE_PREFIX_PATH "C:/Program Files/GTest")
find_package(GTest REQUIRED)

add_executable(occupancygrid_unittests)

target_sources(occupancygrid_unittests PRIVATE occupancygrid_unittests.cpp)
target_include_directories(occupancygrid_unittests PRIVATE "${GTEST_INCLUDE_DIRS}")
target_link_libraries(occupancygrid_unittests
    PRIVATE
        ${GTEST_LIBRARIES}
        ${GTEST_MAIN_LIBRARIES}
        OMap
)
#add_executable(App_RobotVacuum
#    simmain.cpp
#    simentries.h
#    simentries.cpp)

##set(CORRECT_DIR )
#message(STATUS "!!!!!!!!!!!!!!BULLET LIBS: ${BULLET_INCLUDE_DIRS}")
#target_link_directories(App_RobotVacuum PRIVATE ${BULLET_LIBRARY_DIRS} C:/bullet3/lib)
#target_include_directories(App_RobotVacuum PUBLIC ${BULLET_INCLUDE_DIRS} ${BULLET_PHYSICS_SOURCE_DIR}/examples/ExampleBrowser)
#target_link_libraries(App_RobotVacuum ${BULLET_LIBRARIES} RobotVacuumSim BulletExampleBrowserLib)

