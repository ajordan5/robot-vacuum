#include "occupancygrid.h"

OccupancyGrid::OccupancyGrid(double w, double h, double res) : width{w}, height{h}, resolution{res}
{
    int xSize = (int)floor(width/resolution)+1;
    int ySize = (int)floor(height/resolution)+1;

    xCoords.setLinSpaced(xSize, 0, width);
    yCoords.setLinSpaced(ySize, 0, height);
    logLikelihoodMap.resize((xCoords.rows()-1) * (yCoords.rows()-1));

}

Eigen::VectorXd OccupancyGrid::get_x_coords() const
{
    return xCoords;
}

Eigen::VectorXd OccupancyGrid::get_y_coords() const
{
    return yCoords;
}

std::vector<double> OccupancyGrid::get_map() const
{
    return logLikelihoodMap;
}

double OccupancyGrid::get_width() const
{
    return width;
}

double OccupancyGrid::get_height() const
{
    return height;
}

double OccupancyGrid::get_resolution() const
{
    return resolution;
}

int OccupancyGrid::get_cell_index(double x, double y) const
{
    if (x > width || y > height || y <0 || x < 0) return -1;

    int col = (int)floor(x/resolution);
    int row = (int)floor(y/resolution);

    return matrix_to_array_index(xCoords.size()-1, yCoords.size()-1, col, row);

}
