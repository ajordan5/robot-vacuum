#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#define _USE_MATH_DEFINES

#include <vector>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include "occupancygridutils.h"

class OccupancyGrid
{
public:
    OccupancyGrid(double width, double height, double resolution);
    int get_cell_index(double x, double y) const;
    std::pair<double, double> get_cell_center(int idx) const;
    std::vector<int> get_cell_indices_along_ray(double x, double y, double rayAngle, double rayLength) const;
    Eigen::VectorXd get_x_coords() const;
    Eigen::VectorXd get_y_coords() const;
    std::vector<double> get_map() const;
    double get_width() const;
    double get_height() const;
    double get_resolution() const;
    void update_likelihood(double value, double x, double y);

protected:
    double width;
    double height;
    double resolution;
    std::vector<double> logLikelihoodMap;
    Eigen::VectorXd xCoords;
    Eigen::VectorXd yCoords;


};

#endif // OCCUPANCYGRID_H
