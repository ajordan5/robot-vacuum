#ifndef ROBOTVACUUMSIM_H
#define ROBOTVACUUMSIM_H

#include "btBulletDynamicsCommon.h"

class RobotVacuumSim
{
public:
    RobotVacuumSim();
    btDefaultCollisionConfiguration* collisionConfiguration; // = new btDefaultCollisionConfiguration();
    btDiscreteDynamicsWorld* dynamicsWorld;
    btTransform startTransform;

};

#endif // ROBOTVACUUMSIM_H
