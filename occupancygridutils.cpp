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
