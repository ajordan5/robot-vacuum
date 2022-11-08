cmake_minimum_required(VERSION 3.14)

project(RobotVacuumSim LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "C:/bullet3/lib")
#list(APPEND CMAKE_PREFIX_PATH "C:/bullet3/lib/Release")
#list(APPEND CMAKE_PREFIX_PATH "C:/bullet3/build3/vs2010")
list(APPEND CMAKE_PREFIX_PATH "C:/bullet3/src")
list(APPEND CMAKE_PREFIX_PATH "C:/bullet3/examples")
find_package(Bullet REQUIRED )

add_library(RobotVacuumSim STATIC
  robotvacuumsim.cpp
  robotvacuumsim.h
  simmain.cpp
)

target_link_directories(RobotVacuumSim PRIVATE ${BULLET_LIBRARY_DIRS})
target_compile_definitions(RobotVacuumSim PRIVATE ROBOTVACUUMSIM_LIBRARY ${BULLET_DEFINITIONS})
target_include_directories(RobotVacuumSim  PUBLIC ${BULLET_INCLUDE_DIRS} ${BULLET_PHYSICS_SOURCE_DIR}/examples/ExampleBrowser)
target_link_libraries(RobotVacuumSim ${BULLET_LIBRARIES})
