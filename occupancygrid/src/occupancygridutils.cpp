#include "occupancygrid.h"

int matrix_to_array_index(int cols, int rows, int xIdx, int yIdx)
{
    if (xIdx >= cols || yIdx >= rows || xIdx < 0 || yIdx < 0) return -1;
    return yIdx*cols + xIdx;

}

std::pair<int, int> array_to_matrix_index(int cols, int rows, int idx)
{
    if (idx > cols*rows-1) return {-1, -1};

    int xIdx = idx % cols;
    int yIdx = (int)floor(idx/cols);

    return {xIdx, yIdx};
}

double probability_to_logodds(double prob)
{
    if (prob == 1)
        return DBL_MAX;
    return log(prob / (1 - prob));

}

double logodds_to_probability(double logodds)
{
    return exp(logodds) / (1 + exp(logodds));
}

void probability_to_alpha(int* rgbaPixel, double prob)
{
    unsigned char* rgba = reinterpret_cast<unsigned char*>(rgbaPixel);
    unsigned char alpha = 255*prob;
    rgba[3] = alpha;

}

double deg_2_rad(double angleDegrees)
{
    return angleDegrees*M_PI/180;
}

double dist_between_2_points_2D(std::pair<double, double> positionA, std::pair<double, double> positionB)
{
    double diffX{positionA.first - positionB.first};
    double diffY{positionA.second - positionB.second};

    return sqrt(diffX*diffX + diffY*diffY);
}
