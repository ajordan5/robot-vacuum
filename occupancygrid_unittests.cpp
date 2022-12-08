#include "gtest/gtest.h"
#include <iostream>

#include "occupancygrid.h"
#include "lidar.h"
#include "lidarmapper.h"
#include "simutils.h"

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

TEST(OccupancyGridUtils, GivenMatrixIndices_ExpectCorrectArrayIndex)
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

TEST(OccupancyGridUtils, GivenArrayIndex_ExpectCorrectMatrixIndices)
{
    int arrayIdx{6};
    int matrixRows{4};
    int matrixCols{4};

    int goldMatrixX{2};
    int goldMatrixY{1};
    std::pair<int, int> goldIndices{goldMatrixX, goldMatrixY};
    std::pair<int, int> foundIndices = array_to_matrix_index(matrixCols, matrixRows, arrayIdx);

    EXPECT_EQ(goldIndices, foundIndices);
}

TEST(OccupancyGridUtils, GivenArrayIndex_ExpectCorrectMatrixIndices2)
{
    int arrayIdx{12};
    int matrixRows{7};
    int matrixCols{3};

    int goldMatrixX{0};
    int goldMatrixY{4};
    std::pair<int, int> goldIndices{goldMatrixX, goldMatrixY};
    std::pair<int, int> foundIndices = array_to_matrix_index(matrixCols, matrixRows, arrayIdx);

    EXPECT_EQ(goldIndices, foundIndices);
}

TEST(OccupancyGridUtils, GivenArrayIndexOutOfBounds_ExpectNegativeMatrixIndices)
{
    int arrayIdx{21};
    int matrixRows{7};
    int matrixCols{3};

    int goldMatrixX{-1};
    int goldMatrixY{-1};
    std::pair<int, int> goldIndices{goldMatrixX, goldMatrixY};
    std::pair<int, int> foundIndices = array_to_matrix_index(matrixCols, matrixRows, arrayIdx);

    EXPECT_EQ(goldIndices, foundIndices);
}

TEST(OccupancyGridUtils, GivenEvenProbability_ExpectZeroLogOdds)
{
    double prob{0.5};
    double goldLog{0};

    double calculatedLog = probability_to_logodds(prob);

    EXPECT_EQ(goldLog, calculatedLog);
}

TEST(OccupancyGridUtils, GivenZeroLogOdds_ExpectEvenProbability)
{
    double logodds{0};
    double goldProb{0.5};

    double calculatedProb = logodds_to_probability(logodds);

    EXPECT_EQ(goldProb, calculatedProb);
}

TEST(OccupancyGridUtils, GivenSomeProbability_ExpectCorrectLogOdds)
{
    double prob{0.87};
    double goldLog{1.90095876119};

    double calculatedLog = probability_to_logodds(prob);

    EXPECT_NEAR(goldLog, calculatedLog, eps);
}

TEST(OccupancyGridUtils, GivenSomeLogOdds_ExpectCorrectProbability)
{
    double logodds{-3.2};
    double goldProb{0.0391657227968};

    double calculatedProb = logodds_to_probability(logodds);

    EXPECT_NEAR(goldProb, calculatedProb, eps);
}

TEST(OccupancyGridUtils, GivenTwoIdenticalPoints_ExpectZeroDDistance)
{
    std::pair<double, double> positionA{1,1};
    std::pair<double, double> positionB{1,1};

    double goldenDist{0};
    double calculatedDist{dist_between_2_points_2D(positionA, positionB)};

    EXPECT_EQ(goldenDist, calculatedDist);
}

TEST(OccupancyGridUtils, GivenTwoPoints_ExpectCorrect2DDistance)
{
    std::pair<double, double> positionA{1,1};
    std::pair<double, double> positionB{7,3};

    double goldenDist{6.32455532};
    double calculatedDist{dist_between_2_points_2D(positionA, positionB)};

    EXPECT_NEAR(goldenDist, calculatedDist, eps);
}

TEST(OccupancyGridUtils, GivenTwoPoints_ExpectCorrect2DDistance2)
{
    std::pair<double, double> positionA{10.7,15.3};
    std::pair<double, double> positionB{7.4,3.1};

    double goldenDist{12.638433447};
    double calculatedDist{dist_between_2_points_2D(positionA, positionB)};

    EXPECT_NEAR(goldenDist, calculatedDist, eps);
}

TEST(OccupancyGridUtils, GivenHalfProbabilityValue_ExpectCorrectMapToGrayscale)
{
    double prob{0.5};
    unsigned char goldenGrayscale{127};
    std::vector<int> imageBuffer{0, 0, 0, 0, 0};

    probability_to_alpha(&imageBuffer[1], prob);
    unsigned char* rgba = reinterpret_cast<unsigned char*>(&imageBuffer[1]);

    EXPECT_EQ(goldenGrayscale, rgba[3]);
}

TEST(OccupancyGridUtils, GivenFullProbabilityValue_ExpectCorrectMapToGrayscale)
{
    double prob{1.0};
    unsigned char goldenGrayscale{255};
    std::vector<int> imageBuffer{0, 0, 0, 0, 0};

    probability_to_alpha(&imageBuffer[1], prob);
    unsigned char* rgba = reinterpret_cast<unsigned char*>(&imageBuffer[1]);

    EXPECT_EQ(goldenGrayscale, rgba[3]);
}

TEST(OccupancyGridUtils, GivenEmptyProbabilityValue_ExpectCorrectMapToGrayscale)
{
    double prob{0.0};
    unsigned char goldenGrayscale{0};
    std::vector<int> imageBuffer{0, 0, 0, 0, 0};

    probability_to_alpha(&imageBuffer[1], prob);
    unsigned char* rgba = reinterpret_cast<unsigned char*>(&imageBuffer[1]);

    EXPECT_EQ(goldenGrayscale, rgba[3]);
}

TEST(OccupancyGridUtils, GivenSomeProbabilityValue_ExpectCorrectMapToGrayscale)
{
    double prob{0.856};
    unsigned char goldenGrayscale{218};
    std::vector<int> imageBuffer{0, 0, 0, 0, 0};

    probability_to_alpha(&imageBuffer[1], prob);
    unsigned char* rgba = reinterpret_cast<unsigned char*>(&imageBuffer[1]);

    EXPECT_EQ(goldenGrayscale, rgba[3]);
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

TEST(OccupancyGridSetLikelihood, GivenAnInitialGrid_ExpectZeroLikelihoods)
{
    OccupancyGrid og{22, 13, 0.5};

    std::vector<double> map = og.get_map();

    for (int i = 0; i < map.size(); i++)
    {
        EXPECT_EQ(0, map[i]);
    }


}

TEST(OccupancyGridSetLikelihood, GivenAnInitialGridWhenUpdatingValuesOnce_ExpectCorrectValue)
{
    OccupancyGrid og{22, 13, 0.5};
    double value{-20.5};
    og.update_likelihood(value, 10);

    std::vector<double> map = og.get_map();

    EXPECT_EQ(value, map[10]);
}

TEST(OccupancyGridSetLikelihood, GivenAnInitialGridWhenUpdatingValuesOnce_ExpectCorrectValue2)
{
    OccupancyGrid og{4, 4, 1};
    double value{3};
    og.update_likelihood(value, 2);

    std::vector<double> map = og.get_map();

    EXPECT_EQ(value, map[2]);
}

TEST(OccupancyGetCellsOnRay, GivenARayFromAStartingPositionAt0Degrees_ExpectCorrectGridIndices)
{
    OccupancyGrid og{10, 10, 1};
    double x{5.5};
    double y{5.5};
    double rayAngle{0};
    double rayLength{2};

    std::vector<int> goldIndices = {55, 56, 57};
    std::vector<int> foundIndices = og.get_cell_indices_along_ray(x, y, rayAngle, rayLength);

    ASSERT_EQ(goldIndices.size() ,foundIndices.size());

    for (int i=0; i<goldIndices.size(); i++)
    {
        EXPECT_EQ(goldIndices[i], foundIndices[i]);
    }
}

TEST(OccupancyGetCellsOnRay, GivenARayFromAStartingPositionAt90Degrees_ExpectCorrectGridIndices)
{
    OccupancyGrid og{10, 10, 1};
    double x{5.5};
    double y{5.5};
    double rayAngle{M_PI/2};
    double rayLength{2};

    std::vector<int> goldIndices = {55, 65, 75};
    std::vector<int> foundIndices = og.get_cell_indices_along_ray(x, y, rayAngle, rayLength);

    ASSERT_EQ(goldIndices.size() ,foundIndices.size());

    for (int i=0; i<goldIndices.size(); i++)
    {
        EXPECT_EQ(goldIndices[i], foundIndices[i]);
    }
}

TEST(OccupancyGetCellsOnRay, GivenARayFromAStartingPositionAt180Degrees_ExpectCorrectGridIndices)
{
    OccupancyGrid og{10, 10, 1};
    double x{5.5};
    double y{5.5};
    double rayAngle{M_PI};
    double rayLength{2};

    std::vector<int> goldIndices = {55, 54, 53};
    std::vector<int> foundIndices = og.get_cell_indices_along_ray(x, y, rayAngle, rayLength);

    ASSERT_EQ(goldIndices.size() ,foundIndices.size());

    for (int i=0; i<goldIndices.size(); i++)
    {
        EXPECT_EQ(goldIndices[i], foundIndices[i]);
    }
}

TEST(OccupancyGetCellsOnRay, GivenARayFromAStartingPositionDifficultDegrees_ExpectCorrectGridIndices)
{
    OccupancyGrid og{10, 10, 1};
    double x{5.5};
    double y{5.5};
    double rayAngle{3*M_PI/2};
    double rayLength{2};

    std::vector<int> goldIndices = {55, 45, 35};
    std::vector<int> foundIndices = og.get_cell_indices_along_ray(x, y, rayAngle, rayLength);

    ASSERT_EQ(goldIndices.size() ,foundIndices.size());

    for (int i=0; i<goldIndices.size(); i++)
    {
        EXPECT_EQ(goldIndices[i], foundIndices[i]);
    }
}

TEST(OccupancyGetCellsOnRay, GivenARayFromAStartingPositionAtDifficultAngle_ExpectCorrectGridIndices)
{
    OccupancyGrid og{10, 10, 1};
    double x{2.5};
    double y{3.5};
    double rayAngle{-0.588002};
    double rayLength{3};

    std::vector<int> goldIndices = {32, 33, 23, 24, 14};
    std::vector<int> foundIndices = og.get_cell_indices_along_ray(x, y, rayAngle, rayLength);

    ASSERT_EQ(goldIndices.size() ,foundIndices.size());

    for (int i=0; i<goldIndices.size(); i++)
    {
        EXPECT_EQ(goldIndices[i], foundIndices[i]);
    }
}

TEST(OccupancyGetCellsOnRay, GivenARayFromAStartingPositionThatGoesOutOfBounds_ExpectCorrectGridIndicesAndNoneOutOfBounds)
{
    OccupancyGrid og{10, 10, 1};
    double x{1.7};
    double y{8.8};
    double rayAngle{3*M_PI/8};
    double rayLength{7};

    std::vector<int> goldIndices = {81, 91, 92};
    std::vector<int> foundIndices = og.get_cell_indices_along_ray(x, y, rayAngle, rayLength);

    ASSERT_EQ(goldIndices.size() ,foundIndices.size());

    for (int i=0; i<goldIndices.size(); i++)
    {
        EXPECT_EQ(goldIndices[i], foundIndices[i]);
    }
}

TEST(OccupancyGetCellsOnRay, GivenARayFromAStartingPositionThatSlightlyEntersACell_ExpectGridIndicesToIncludeThatCell)
{
    OccupancyGrid og{10, 10, 1};
    double x{1.249};
    double y{1.1};
    double rayAngle{0};
    double rayLength{0.752};

    std::vector<int> goldIndices = {11, 12};
    std::vector<int> foundIndices = og.get_cell_indices_along_ray(x, y, rayAngle, rayLength);

    ASSERT_EQ(goldIndices.size() ,foundIndices.size());

    for (int i=0; i<goldIndices.size(); i++)
    {
        EXPECT_EQ(goldIndices[i], foundIndices[i]);
    }
}

TEST(OccupancyGetCellsOnRay, GivenARayFromAStartingPositionThatNearlyEntersACell_ExpectGridIndicesToNotIncludeThatCell)
{
    OccupancyGrid og{10, 10, 1};
    double x{1.25};
    double y{1.1};
    double rayAngle{0};
    double rayLength{0.749};

    std::vector<int> goldIndices = {11};
    std::vector<int> foundIndices = og.get_cell_indices_along_ray(x, y, rayAngle, rayLength);

    ASSERT_EQ(goldIndices.size() ,foundIndices.size());

    for (int i=0; i<goldIndices.size(); i++)
    {
        EXPECT_EQ(goldIndices[i], foundIndices[i]);
    }
}

TEST(OccupancyGridGetCellCenter, GivenOccupancyGridIndices_ExpectCorrectGridCenter)
{
    OccupancyGrid og{3, 3, 1};
    int idx{4};
    std::pair<double, double> goldCenter{1.5, 1.5};
    std::pair<double, double> foundCenter = og.get_cell_center(idx);

    EXPECT_EQ(goldCenter, foundCenter);

}

TEST(OccupancyGridGetCellCenter, GivenOccupancyGridIndices_ExpectCorrectGridCenter2)
{
    OccupancyGrid og{5, 5, 0.5};
    int idx{22};
    std::pair<double, double> goldCenter{1.25, 1.25};
    std::pair<double, double> foundCenter = og.get_cell_center(idx);

    EXPECT_EQ(goldCenter, foundCenter);

}

TEST(OccupancyGridGetCellCenter, GivenOccupancyGridIndices_ExpectCorrectGridCenter3)
{
    OccupancyGrid og{3, 3, 0.3};
    int idx{9};
    std::pair<double, double> goldCenter{2.85, 0.15};
    std::pair<double, double> foundCenter = og.get_cell_center(idx);

    EXPECT_NEAR(goldCenter.first, foundCenter.first, eps);
    EXPECT_NEAR(goldCenter.second, foundCenter.second, eps);

}

TEST(Lidar, GivenInitLidar_ExpectEquallySpacedRaysAtCorrectAngles)
{
    btDynamicsWorld* world{nullptr};
    Lidar lidar(world, 200, 60, 11);

    Eigen::Vector<double, 11> goldAngles;
    goldAngles << -30, -24, -18, -12, -6, 0, 6, 12, 18, 24, 30;

    Eigen::VectorXd foundAngles = lidar.get_angles_degrees();
    vectors_equal(goldAngles, foundAngles);

}

TEST(Lidar, GivenInitLidar_ExpectEquallySpacedRaysAtCorrectAngles2)
{
    btDynamicsWorld* world{nullptr};
    Lidar lidar(world, 200, 20, 3);

    Eigen::Vector<double, 3> goldAngles;
    goldAngles << -10, 0, 10;

    Eigen::VectorXd foundAngles = lidar.get_angles_degrees();
    vectors_equal(goldAngles, foundAngles);

}

TEST(Lidar, GivenInitLidarWith360View_ExpectRaysToNotOverlapAndBeEquallySpaced)
{
    btDynamicsWorld* world{nullptr};
    Lidar lidar(world, 200, 360, 10);

    Eigen::Vector<double, 10> goldAngles;
    goldAngles << 0, 36, 72, 108, 144, 180, 216 , 252, 288, 324;

    Eigen::VectorXd foundAngles = lidar.get_angles_degrees();
    vectors_equal(goldAngles, foundAngles);

}

TEST(Lidar, GivenInitLidarWithNegativeView_ExpectInvalidArgException)
{
    btDynamicsWorld* world{nullptr};

    EXPECT_THROW({
            try
            {
                Lidar lidar(world, 200, -360, 10);
            }
            catch( const std::invalid_argument& e )
            {
                EXPECT_STREQ( "Lidar parameters must be positive", e.what() );
                throw;
            }
        }, std::invalid_argument );


}

TEST(Lidar, GivenInitLidarWithNegativeNumRays_ExpectInvalidArgException)
{
    btDynamicsWorld* world{nullptr};

    EXPECT_THROW({
            try
            {
                Lidar lidar(world, 200, 60, -10);
            }
            catch( const std::invalid_argument& e )
            {
                EXPECT_STREQ( "Lidar parameters must be positive", e.what() );
                throw;
            }
        }, std::invalid_argument );


}

TEST(Lidar, GivenInitLidarWithNegativeRange_ExpectInvalidArgException)
{
    btDynamicsWorld* world{nullptr};

    EXPECT_THROW({
            try
            {
                Lidar lidar(world, -200, 360, 10);
            }
            catch( const std::invalid_argument& e )
            {
                EXPECT_STREQ( "Lidar parameters must be positive", e.what() );
                throw;
            }
        }, std::invalid_argument );


}

TEST(SimUtilsDeg2Rad, GivenZeroAngleInDegrees_ExpectCorrectZeroAngleInRadians)
{
    double angleDegrees{0};
    double goldenRadians{0};

    double calulatedRadians = deg_2_rad(angleDegrees);

    EXPECT_EQ(goldenRadians, calulatedRadians);
}

TEST(SimUtilsDeg2Rad, GivenAngleInDegrees_ExpectCorrectAngleInRadians)
{
    double angleDegrees{20};
    double goldenRadians{M_PI/9};

    double calulatedRadians = deg_2_rad(angleDegrees);

    EXPECT_NEAR(goldenRadians, calulatedRadians, eps);
}

TEST(SimUtilsDeg2Rad, GivenAngleInDegrees_ExpectCorrectAngleInRadians2)
{
    double angleDegrees{-15};
    double goldenRadians{-M_PI/12};

    double calulatedRadians = deg_2_rad(angleDegrees);

    EXPECT_EQ(goldenRadians, calulatedRadians);
}

TEST(SimUtilsRayRotate, GivenLocalCoordinateFrameAtIdentityWithRayAnglePair_ExpectCorrectLocalRayRotation)
{
    double angleDegrees{-15};
    btVector3 ray{10, 0, 0};
    btTransform localFrame;
    localFrame.setIdentity();

    btVector3 goldenRay{9.659, -2.588, 0};
    btVector3 rotatedRay = rotate_ray_local(localFrame, ray, angleDegrees);

    for (int i=0; i<3; ++i)
    {
        EXPECT_NEAR(goldenRay[i], rotatedRay[i], .001);
    }

}

TEST(SimUtilsRayRotate, GivenRotatedLocalCoordinateFrameWithRayAnglePair_ExpectCorrectLocalRayRotation)
{
    double angleDegrees{-15};
    btVector3 ray{10, 0, 0};
    btTransform localFrame;
    localFrame.setIdentity();
    btVector3 axis(0, 0, 1);
    localFrame.setRotation(btQuaternion(axis, M_PI_4));

    btVector3 goldenRay{8.660, 5, 0};
    btVector3 rotatedRay = rotate_ray_local(localFrame, ray, angleDegrees);

    for (int i=0; i<3; ++i)
    {
        EXPECT_NEAR(goldenRay[i], rotatedRay[i], .001);
    }

}

TEST(SimUtilsRayRotate, GivenNegativeRotatedLocalCoordinateFrameWithRayAnglePair_ExpectCorrectLocalRayRotation)
{
    double angleDegrees{-15};
    btVector3 ray{10, 0, 0};
    btTransform localFrame;
    localFrame.setIdentity();
    btVector3 axis(0, 0, 1);
    localFrame.setRotation(btQuaternion(axis, -M_PI_2));

    btVector3 goldenRay{-2.588, -9.659, 0};
    btVector3 rotatedRay = rotate_ray_local(localFrame, ray, angleDegrees);

    for (int i=0; i<3; ++i)
    {
        EXPECT_NEAR(goldenRay[i], rotatedRay[i], .001);
    }

}

TEST(SimUtilsRayRotate, GivenTransformedLocalCoordinateFrameWithRayAnglePair_ExpectCorrectLocalRayRotation)
{
    double angleDegrees{-15};
    btVector3 ray{10, 0, 0};
    btTransform localFrame;
    localFrame.setIdentity();
    btVector3 axis(0, 0, 1);
    localFrame.setRotation(btQuaternion(axis, M_PI/10));
    localFrame.setOrigin({1,2,3});

    btVector3 goldenRay{10.986, 2.523, 3};
    btVector3 rotatedRay = rotate_ray_local(localFrame, ray, angleDegrees);

    for (int i=0; i<3; ++i)
    {
        EXPECT_NEAR(goldenRay[i], rotatedRay[i], .001);
    }

}

TEST(SimUtilsRayRotate, GivenTransformedLocalCoordinateFrameWithRayAnglePair_ExpectCorrectLocalRayRotation2)
{
    double angleDegrees{5};
    btVector3 ray{5, 0, 0};
    btTransform localFrame;
    localFrame.setIdentity();
    btVector3 axis(0, 0, 1);
    localFrame.setRotation(btQuaternion(axis, M_PI/18));
    localFrame.setOrigin({-5,2,6});

    btVector3 goldenRay{-0.1704, 3.294, 6};
    btVector3 rotatedRay = rotate_ray_local(localFrame, ray, angleDegrees);

    for (int i=0; i<3; ++i)
    {
        EXPECT_NEAR(goldenRay[i], rotatedRay[i], .001);
    }

}

TEST(SimUtilsIsArrowkey, GivenArrowKey_ExpectTrue)
{
    std::vector<int> goldenKeys{16777234, 16777235, 16777236, 16777237};

    for (int key : goldenKeys)
    {
        EXPECT_TRUE(is_arrowkey(key));
    }

}


TEST(SimUtilsIsArrowkey, GivenNonArrowKeys_ExpectFalse)
{
    std::vector<int> nonArrowKeys{1, 2, 3, 36, 200};

    for (int key : nonArrowKeys)
    {
        EXPECT_FALSE(is_arrowkey(key));
    }

}

TEST(SimUtilsHeading, GivenIdentityMatrix_ExpectZeroHeading)
{
    btTransform trans;
    trans.setIdentity();
    double goldenHeading{0};
    double calculatedHeading{get_heading_of_z_rot_matrix(trans)};

    EXPECT_EQ(goldenHeading, calculatedHeading);
}

TEST(SimUtilsHeading, GivenMatrixRotatedAboutZ_ExpectCorrectHeading)
{
    btTransform trans;
    trans.setIdentity();
    btVector3 axis{0,0,1};
    trans.setRotation(btQuaternion(axis, M_PI/10));
    double goldenHeading{M_PI/10};
    double calculatedHeading{get_heading_of_z_rot_matrix(trans)};

    EXPECT_NEAR(goldenHeading, calculatedHeading, 0.0001);
}

TEST(SimUtilsHeading, GivenMatrixRotatedAboutZ180Degrees_ExpectCorrectHeading)
{
    btTransform trans;
    trans.setIdentity();
    btVector3 axis{0,0,1};
    trans.setRotation(btQuaternion(axis, M_PI-.00001));
    double goldenHeading{M_PI};
    double calculatedHeading{get_heading_of_z_rot_matrix(trans)};

    EXPECT_NEAR(goldenHeading, calculatedHeading, 0.0001);
}

TEST(SimUtilsHeading, GivenMatrixRotatedAboutZ90Degrees_ExpectCorrectHeading)
{
    btTransform trans;
    trans.setIdentity();
    btVector3 axis{0,0,1};
    trans.setRotation(btQuaternion(axis, M_PI_2));
    double goldenHeading{M_PI_2};
    double calculatedHeading{get_heading_of_z_rot_matrix(trans)};

    EXPECT_NEAR(goldenHeading, calculatedHeading, 0.0001);
}

TEST(SimUtilsHeading, GivenMatrixRotatedAboutZByLargeAngle_ExpectCorrectHeading)
{
    btTransform trans;
    trans.setIdentity();
    btVector3 axis{0,0,1};
    trans.setRotation(btQuaternion(axis, 15*M_PI/8));
    double goldenHeading{-M_PI/8};
    double calculatedHeading{get_heading_of_z_rot_matrix(trans)};

    EXPECT_NEAR(goldenHeading, calculatedHeading, 0.0001);
}

TEST(SimUtilsHeading, GivenMatrixRotatedAboutZByNegativeAngle_ExpectCorrectHeading)
{
    btTransform trans;
    trans.setIdentity();
    btVector3 axis{0,0,1};
    trans.setRotation(btQuaternion(axis, -M_PI/8));
    double goldenHeading{-M_PI/8};
    double calculatedHeading{get_heading_of_z_rot_matrix(trans)};

    EXPECT_NEAR(goldenHeading, calculatedHeading, 0.0001);
}

TEST(SimUtilsHeading, GivenMatrixRotatedAboutZByLargeNegativeAngle_ExpectCorrectHeading)
{
    btTransform trans;
    trans.setIdentity();
    btVector3 axis{0,0,1};
    trans.setRotation(btQuaternion(axis, -26*M_PI/8));
    double goldenHeading{3*M_PI_4};
    double calculatedHeading{get_heading_of_z_rot_matrix(trans)};

    EXPECT_NEAR(goldenHeading, calculatedHeading, 0.0001);
}

