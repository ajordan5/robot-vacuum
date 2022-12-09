add_executable(App_RobotVacuum
    src/main.cpp
    src/mainwindow.cpp
    src/mappainter.cpp
    src/mainwindow.ui
    include/mainwindow.h
    include/mappainter.h
)

target_compile_definitions(App_RobotVacuum PRIVATE ${BULLET_DEFINITIONS})
target_include_directories(App_RobotVacuum
    PRIVATE
    ${BULLET_INCLUDE_DIRS}
    include
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
