#ifndef OCCUPANCYGRIDUTILS_H
#define OCCUPANCYGRIDUTILS_H

#include <vector>

int matrix_to_array_index(int cols, int rows, int xIdx, int yIdx);
std::pair<int, int> array_to_matrix_index(int cols, int rows, int idx);
double probability_to_logodds(double prob);
double logodds_to_probability(double logodds);
void probability_to_alpha(int* rgbaPixel, double prob);
double deg_2_rad(double angleDegrees);
double dist_between_2_points_2D(std::pair<double, double> positionA, std::pair<double, double> positionB);

#endif // OCCUPANCYGRIDUTILS_H
