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

std::vector<int> OccupancyGrid::get_cell_indices_along_ray(double x, double y, double rayAngle, double rayLength) const
{
    double checkIncrement{resolution/3};
    double distanceTraveledAlongRay{0};
    double checkX;
    double checkY;
    int indexPrev{-1};
    std::vector<int> indicesReturn;

    while (distanceTraveledAlongRay <= rayLength)
    {
        checkX = x + distanceTraveledAlongRay * cos(rayAngle);
        checkY = y + distanceTraveledAlongRay * sin(rayAngle);

        int cellIndex = get_cell_index(checkX, checkY);

        if (cellIndex == -1)
            return indicesReturn;

        if (cellIndex != indexPrev)
        {
            indicesReturn.push_back(cellIndex);
            indexPrev = cellIndex;
        }

        if (rayLength - distanceTraveledAlongRay < checkIncrement && rayLength != distanceTraveledAlongRay)
            distanceTraveledAlongRay = rayLength;
        else
            distanceTraveledAlongRay = distanceTraveledAlongRay + checkIncrement;

    }

    return indicesReturn;
}

std::pair<double, double> OccupancyGrid::get_cell_center(int idx) const
{
    std::pair<int, int> matrixIndices = array_to_matrix_index(xCoords.size()-1, yCoords.size()-1, idx);
    double xCenter = xCoords(matrixIndices.first ) + resolution/2;
    double yCenter = yCoords(matrixIndices.second) + resolution/2;
    return {xCenter, yCenter};
}

void OccupancyGrid::update_likelihood(double value, int idx)
{
    if (idx >= 0 && idx < logLikelihoodMap.size()) logLikelihoodMap[idx] += value;
}

void OccupancyGrid::set_initial_likelihood(double initValue)
{
    std::fill(logLikelihoodMap.begin(), logLikelihoodMap.end(), initValue);
}
