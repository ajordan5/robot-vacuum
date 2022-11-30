#ifndef WORLD_H
#define WORLD_H

#include <QtCore>
#include <Qt3DCore/qentity.h>
#include "ground.h"
#include "vacuum.h"
#include "obstacles.h"
#include "lidarmapper.h"

class World
{
public:
    World(Qt3DCore::QEntity* rootEntity, double simTimeStep);
    ~World();

    void step();
    void key_press(int key) const;
    void key_release() const;

private:
    void init_physics();
    void create_world();
    void pass_measurements_to_map();

    Qt3DCore::QEntity* mRootEntity;
    Ground* mGround;
    Vacuum* mVacuum;
    Obstacles* mObstacles;
    LidarMapper* map;
    double timeStep;

    btBroadphaseInterface* broadphaseInterface;
    btDefaultCollisionConfiguration* defaultCollisionConfig;
    btCollisionDispatcher* collisionDispatcher;
    btSequentialImpulseConstraintSolver* seqImpConstraintSolver;
    btDiscreteDynamicsWorld* dynamicsWorld;
};

#endif // WORLD_H
