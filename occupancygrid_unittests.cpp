#include "gtest/gtest.h"
#include "occupancygrid.h"
#include <iostream>

#define eps 0.00000001

void vectors_equal(Eigen::VectorXd v1, Eigen::VectorXd v2)
{
    EXPECT_EQ(v1.size(), v2.size());

    for (int i = 0; i <v1.size(); i++)
    {
        EXPECT_NEAR(v1(i), v2(i), eps);
    }
}

TEST(OccupancyGridConstructor, GivenAnInitializedOccupancyGrid_ExpectCorrectDimensions)
{
    OccupancyGrid og{20, 20, 1};
    double width = og.get_width();
    double height = og.get_height();
    double res = og.get_resolution();

    EXPECT_EQ(20, width);
    EXPECT_EQ(20, height);
    EXPECT_EQ(1, res);
}

TEST(OccupancyGridConstructor, GivenAnInitializedOccupancyGrid_ExpectCorrectDimensions2)
{
    OccupancyGrid og{55, 12, 0.75};
    double width = og.get_width();
    double height = og.get_height();
    double res = og.get_resolution();

    EXPECT_EQ(55, width);
    EXPECT_EQ(12, height);
    EXPECT_EQ(0.75, res);
}

TEST(OccupancyGridConstructor, GivenAnInitializedOccupancyGrid_ExpectCorrectMapSize)
{
    OccupancyGrid og{55, 12, 0.75};
    std::vector<double> map = og.get_map();
    Eigen::VectorXd xCoords = og.get_x_coords();

    int goldSize{1168};

    EXPECT_EQ(goldSize, map.size());
}

TEST(OccupancyGridConstructor, GivenAnInitializedOccupancyGrid_ExpectCorrectMapSize2)
{
    OccupancyGrid og{20, 30, 0.1};
    std::vector<double> map = og.get_map();
    Eigen::VectorXd xCoords = og.get_x_coords();

    int goldSize{60000};

    EXPECT_EQ(goldSize, map.size());
}

TEST(OccupancyGridConstructor, GivenAnInitializedOccupancyGrid_ExpectCorrectXCoords)
{
    OccupancyGrid og{1, 2, 0.1};
    std::vector<double> map = og.get_map();
    Eigen::VectorXd xCoords = og.get_x_coords();

    Eigen::Vector<double, 11> goldCoords;
    goldCoords << 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1;

    vectors_equal(xCoords, goldCoords);
}

TEST(OccupancyGridConstructor, GivenAnInitializedOccupancyGrid_ExpectCorrectXCoords2)
{
    OccupancyGrid og{2, 2, 0.5};
    std::vector<double> map = og.get_map();
    Eigen::VectorXd xCoords = og.get_x_coords();

    Eigen::Vector<double, 5> goldCoords;
    goldCoords << 0, 0.5, 1, 1.5, 2;

    vectors_equal(xCoords, goldCoords);
}

TEST(OccupancyGridConstructor, GivenAnInitializedOccupancyGrid_ExpectCorrectYCoords)
{
    OccupancyGrid og{1, 2, 0.25};
    std::vector<double> map = og.get_map();
    Eigen::VectorXd yCoords = og.get_y_coords();

    Eigen::Vector<double, 9> goldCoords;
    goldCoords << 0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2;

    vectors_equal(yCoords, goldCoords);
}

TEST(OccupancyGridConstructor, GivenAnInitializedOccupancyGrid_ExpectCorrectYCoords2)
{
    OccupancyGrid og{1, 1, 0.2};
    std::vector<double> map = og.get_map();
    Eigen::VectorXd yCoords = og.get_y_coords();

    Eigen::Vector<double, 6> goldCoords;
    goldCoords << 0, 0.2, 0.4, 0.6, 0.8, 1;

    vectors_equal(yCoords, goldCoords);
}

TEST(OccupancyGridGetUtils, GivenMatrixIndices_ExpectCorrectArrayIndex)
{
    int matrixX{3};
    int matrixY{4};
    int matrixRows{5};
    int matrixCols{7};

    int goldIdx{31};
    int gridIdx{matrix_to_array_index(matrixCols, matrixRows, matrixX, matrixY)};

    EXPECT_EQ(goldIdx, gridIdx);
}

TEST(OccupancyGridUtils, GivenMatrixIndicesAtZero_ExpectCorrectArrayIndexAtZero)
{
    int matrixX{0};
    int matrixY{0};
    int matrixRows{5};
    int matrixCols{7};

    int goldIdx{0};
    int gridIdx{matrix_to_array_index(matrixCols, matrixRows, matrixX, matrixY)};

    EXPECT_EQ(goldIdx, gridIdx);
}

TEST(OccupancyGridUtils, GivenMatrixIndicesOutOfBounds_ExpectNegativeOneIndex)
{
    int matrixX{200};
    int matrixY{0};
    int matrixRows{100};
    int matrixCols{200};

    int goldIdx{-1};
    int gridIdx{matrix_to_array_index(matrixCols, matrixRows, matrixX, matrixY)};

    EXPECT_EQ(goldIdx, gridIdx);
}

TEST(OccupancyGridGetIndex, GivenAPositionInWorldCoordinates_ExpectCorrectGridIndices)
{
    OccupancyGrid og{1, 1, 0.2};
    double x{0.25};
    double y{0.33};

    int goldIdx{6};
    int gridIdx{og.get_cell_index(x, y)};

    EXPECT_EQ(goldIdx, gridIdx);
}

TEST(OccupancyGridGetIndex, GivenAPositionInWorldCoordinates_ExpectCorrectGridIndices2)
{
    OccupancyGrid og{7, 4, 0.33};
    double x{1.5};
    double y{2.4};

    int goldIdx{151};
    int gridIdx{og.get_cell_index(x, y)};

    EXPECT_EQ(goldIdx, gridIdx);
}

TEST(OccupancyGridGetIndex, GivenAPositionOutsideWorldCoordinates_ExpectNegativeOneGridIndex)
{
    OccupancyGrid og{22, 13, 0.75};
    double x{33};
    double y{2.4};

    int goldIdx{-1};
    int gridIdx{og.get_cell_index(x, y)};

    EXPECT_EQ(goldIdx, gridIdx);
}
