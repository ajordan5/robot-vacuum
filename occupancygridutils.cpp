int matrix_to_array_index(int cols, int rows, int xIdx, int yIdx)
{
    if (xIdx >= cols || yIdx >= rows || xIdx < 0 || yIdx < 0) return -1;
    return yIdx*cols + xIdx;

}
