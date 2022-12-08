add_library( OMap STATIC
  src/occupancygrid.cpp
  src/occupancygridutils.cpp
  src/lidarmapper.cpp

)

target_include_directories(OMap PUBLIC include)
target_link_libraries(OMap Eigen3::Eigen)
