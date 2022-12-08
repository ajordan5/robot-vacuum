# Robot Vacuum Occupancy Gridmap

![](https://github.com/MEEN-570-Fall-2022/final-project-ajordan5/blob/main/doc/demo.gif)

This package implements [occupancy grid mapping](https://www.researchgate.net/profile/Luiz-Goncalves-4/publication/261174154_Probabilistic_robotic_grid_mapping_based_on_occupancy_and_elevation_information/links/57da887008ae72d72ea33dd3/Probabilistic-robotic-grid-mapping-based-on-occupancy-and-elevation-information.pdf) for a simulated robotic vacuum. Physics simulation is provided by [Bullet3](https://github.com/bulletphysics/bullet3) and graphics are implemented with [Qt3D](https://doc.qt.io/qt-6/qt3d-index.html). The robot is user controlled with either a Playstation DualSense contoller or keyboard arrow keys. Bullet3 raytracing is used to simulate LIDAR rays, and ray measurements are used to update grid cell occupancy values.

## Dependencies
* [Eigen3](https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html)
* [Bullet3](https://github.com/bulletphysics/bullet3)
* Qt6 (recommended to build this package with Qt Creator)
* [DualSense-Windows](https://github.com/Ohjurot/DualSense-Windows) (optional, see below)
  * Per DualSense-Windows instructions, install and link to `ds5w.lib` and put `ds5w.dll` in your build folder.
  
## Controls

![alt text](https://github.com/MEEN-570-Fall-2022/final-project-ajordan5/blob/main/doc/control.JPG)

### The robot can be controlled with either a connected Playstation DualSense controller or keyboard arrowkeys. If the user wishes to control with only keyboard and forgo the DualSense dependency, checkout and build the `keyboard` branch.

**Warning**: Turbo speed is fun, but degrades mapping quality!
