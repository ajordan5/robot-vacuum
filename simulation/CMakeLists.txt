add_library(RobotVacuumSim STATIC
  src/vacuum.cpp
  src/ground.cpp
  src/obstacles.cpp
  src/lidar.cpp
  src/world.cpp
  src/simutils.cpp

)

target_link_directories(RobotVacuumSim PRIVATE ${BULLET_LIBRARY_DIRS})
target_compile_definitions(RobotVacuumSim PRIVATE ROBOTVACUUMSIM_LIBRARY ${BULLET_DEFINITIONS})
target_include_directories(RobotVacuumSim  PUBLIC ${BULLET_INCLUDE_DIRS} include)
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
