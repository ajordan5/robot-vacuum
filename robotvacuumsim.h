#ifndef ROBOTVACUUMSIM_H
#define ROBOTVACUUMSIM_H

#include "btBulletDynamicsCommon.h"

class RobotVacuumSim
{
public:
    RobotVacuumSim();
    btDefaultCollisionConfiguration* collisionConfiguration;
    btDiscreteDynamicsWorld* dynamicsWorld;
};

#endif // ROBOTVACUUMSIM_H
