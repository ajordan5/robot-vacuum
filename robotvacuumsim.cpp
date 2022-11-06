#include "robotvacuumsim.h"

RobotVacuumSim::RobotVacuumSim()
{
    // I can use types from the headers, but creating a new pointer to any of the object throws an error
    btCollisionDispatcher* dispatcher; // = new btCollisionDispatcher(collisionConfiguration);
    startTransform.setIdentity();
    btScalar mass(1.f);
    btVector3 localInertia(0, 0, 0);
    btCollisionShape* colShape; // = new btSphereShape(btScalar(1.));
}
