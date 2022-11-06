cmake_minimum_required(VERSION 3.14)

project(RobotVacuumSim LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "C:/bullet3/lib/Debug")
list(APPEND CMAKE_PREFIX_PATH "C:/bullet3/build3")
list(APPEND CMAKE_PREFIX_PATH "C:/bullet3/src")
find_package(Bullet) # REQUIRED )

add_library(RobotVacuumSim SHARED
  robotvacuumsim.cpp
  robotvacuumsim.h
)

target_link_libraries(RobotVacuumSim ${BULLET_LIBRARIES})

target_compile_definitions(RobotVacuumSim PRIVATE ROBOTVACUUMSIM_LIBRARY)
target_include_directories(RobotVacuumSim  PUBLIC ${BULLET_INCLUDE_DIRS})
