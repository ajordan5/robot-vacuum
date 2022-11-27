#ifndef LIDAR_H
#define LIDAR_H

#include <vector>
#include <Eigen/Dense>
#include "btBulletDynamicsCommon.h"

class Lidar
{
public:
    Lidar(btDynamicsWorld* ownerWorld, double range, double viewAngleDegrees, int numRays);
    double get_max_range(){return maxRange;}
    std::pair<Eigen::VectorXd, Eigen::VectorXd> get_intersections_angle_and_distance(btTransform state);
    Eigen::VectorXd get_angles_degrees(){return anglesDegrees;};
    void draw_rays();

private:
    double maxRange;
    double fieldOfViewDegrees;
    int numberOfRays;
    btDynamicsWorld* world;

    Eigen::VectorXd anglesDegrees;
    Eigen::VectorXd lengths;

    btVector3 to;
};

#endif // LIDAR_H
