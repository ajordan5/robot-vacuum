#include "lidarmapper.h"

LidarMapper::LidarMapper(double maxRange, double width, double height, double resolution)
    : lidarRange{maxRange}, mapResolution{resolution}
{
    logOddsFree = probability_to_logodds(probabiltyFree);
    logOddsOccup = probability_to_logodds(probabiltyPrior);
    logOddsOccup = probability_to_logodds(probabiltyOccup);

    gridMap = new OccupancyGrid(width, height, resolution);
    mapWidth = gridMap->get_x_coords().size()-1;
    mapHeight = gridMap->get_y_coords().size()-1;
    imageBuffer.resize(gridMap->get_map().size());
    set_inital_image();

}

void LidarMapper::add_measurements_to_map(const std::pair<Eigen::VectorXd, Eigen::VectorXd>& rayAngleLengthPairs, const VehicleState& state)
{
    for (int i = 0; i < rayAngleLengthPairs.first.size(); ++i)
    {
        double rayAngle{rayAngleLengthPairs.first(i)};
        double rayLength{rayAngleLengthPairs.second(i)};
        double totalAngle{deg_2_rad(rayAngle) + state.heading};

        std::vector<int> cellsToUpdate = gridMap->get_cell_indices_along_ray(state.x, state.y, totalAngle, rayLength);
        integrate_cells_along_ray(cellsToUpdate, state, rayLength);
    }

}

void LidarMapper::integrate_cells_along_ray(const std::vector<int>& cellIndices, const VehicleState& state, double rayLength)
{
    for (int index : cellIndices)
    {
        std::pair<double, double> cellCenter = gridMap->get_cell_center(index);
        double distance = dist_between_2_points_2D(cellCenter, {state.x, state.y});
        double logOddsUpdate = inverse_lidar_model(distance, rayLength);
        gridMap->update_likelihood(logOddsUpdate, index);
        update_image_with_likelihood(gridMap->get_map()[index], index);


    }
}

double LidarMapper::inverse_lidar_model(double cellDistance, double rayLength)
{
    if (rayLength == lidarRange) return logOddsFree;
    else if (cellDistance <= rayLength - mapResolution/2) return logOddsFree;
    else if (cellDistance > rayLength - mapResolution/2 && cellDistance < rayLength + mapResolution/2) return logOddsOccup;
    else return logOddsPrior;

}

const unsigned char* LidarMapper::get_image() const
{
    return reinterpret_cast<const unsigned char*>(imageBuffer.data());
}

void LidarMapper::set_inital_image()
{
    int initValue{0};
    unsigned char* rgba = reinterpret_cast<unsigned char*>(&initValue);
    rgba[3] = 127;
    rgba[2] = 0;
    rgba[1] = 0;
    rgba[0] = 255;
    std::fill(imageBuffer.begin(), imageBuffer.end(), initValue);

}

void LidarMapper::update_image_with_likelihood(double likelihood, int index)
{
    double cellProbability{logodds_to_probability(likelihood)};
    probability_to_alpha(&imageBuffer[index], cellProbability);
}
