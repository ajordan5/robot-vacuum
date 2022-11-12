#ifndef OCCUPANCYGRIDUTILS_H
#define OCCUPANCYGRIDUTILS_H

#include <vector>

int matrix_to_array_index(int cols, int rows, int xIdx, int yIdx);
std::pair<int, int> array_to_matrix_index(int cols, int rows, int idx);

#endif // OCCUPANCYGRIDUTILS_H
