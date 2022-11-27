#include "lidar.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "bulletutils.h"
#include <QDebug>

Lidar::Lidar(btDynamicsWorld* ownerWorld, double range, double viewAngleDeg, int numRays)
    : maxRange{range}, fieldOfViewDegrees{viewAngleDeg}, numberOfRays{numRays}, world{ownerWorld}, to{-maxRange, 0, 0}
{
    if (range <= 0 || viewAngleDeg <= 0 || numRays <= 0)
    {
        throw std::invalid_argument( "Lidar parameters must be positive" );
    }
    if (fieldOfViewDegrees == 360)
    {
        double increment = 360/numRays;
        anglesDegrees.setLinSpaced(numRays, 0, 360-increment);
    }
    else
        anglesDegrees.setLinSpaced(numRays, -fieldOfViewDegrees/2, fieldOfViewDegrees/2);

    lengths.resize(numRays);

}

std::pair<Eigen::VectorXd, Eigen::VectorXd> Lidar::get_intersections_angle_and_distance(btTransform state)
{

    btVector3 axis(0, 0, 1);
    for (int i = 0; i< anglesDegrees.size(); i++)
    {
        double angle{anglesDegrees(i)};
        btVector3 rayFrom = state.getOrigin();
        btVector3 rayTo = rotate_ray_local(state, to, angle);

        btCollisionWorld::ClosestRayResultCallback closestResults(rayFrom, rayTo);
        closestResults.m_flags |= btTriangleRaycastCallback::kF_FilterBackfaces;
        world->rayTest(rayFrom, rayTo, closestResults);

        lengths(i) = closestResults.m_closestHitFraction * maxRange;

    }

    return {anglesDegrees, lengths};

}
