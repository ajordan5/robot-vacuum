#ifndef LIDARMAPPER_H
#define LIDARMAPPER_H

#include "occupancygrid.h"

struct VehicleState
{
    double x;
    double y;
    double heading;
};

class LidarMapper
{
public:
    LidarMapper(double maxRange, double width, double height, double resolution);
    void add_measurements_to_map(const std::pair<Eigen::VectorXd, Eigen::VectorXd>& rayAngleLengthPairs, const VehicleState& state);
    const unsigned char* get_image() const;
    int get_num_rows() {return numMapCols;}
    int get_num_cols() {return numMapRows;}
    void set_initial_image();
    int percent_environment_mapped();

private:
    void integrate_cells_along_ray(const std::vector<int>& cellIndices, const VehicleState& state, double rayLength);
    double inverse_lidar_model(double cellDistance, double rayLength);
    void update_image_with_likelihood(double likelihood, int index);

    OccupancyGrid* gridMap;
    double probabiltyFree{0.35};
    double probabiltyPrior{0.5};
    double probabiltyOccup{0.85};
    double logOddsFree;
    double logOddsPrior;
    double logOddsOccup;
    double lidarRange;
    double mapResolution;
    int numMapRows;
    int numMapCols;
    int initValue;
    std::vector<int> imageBuffer;
};

#endif // LIDARMAPPER_H
