#ifndef OBSTACLES_H
#define OBSTACLES_H

#include "btBulletDynamicsCommon.h"
#include <QMatrix4x4>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <QColor>

#include "simutils.h"

class Obstacles
{
public:
    Obstacles(btDynamicsWorld* ownerWorld);
    btRigidBody** get_obstacle_body() {return m_bodies;}
    Qt3DCore::QEntity** getQEntity() {return mEntities;}

private:
    btDynamicsWorld* m_ownerWorld;
    btCollisionShape* m_shapes[8];
    btRigidBody* m_bodies[8];
    double wallThickness{10};
    double wallHeight{50};
    double outerWallLength{400};

    btScalar btMat[16];
    btTransform trans;

    Qt3DCore::QTransform *mTransforms[8];
    Qt3DCore::QEntity *mEntities[8];

    void setup_physics();
    void setup_graphics();
    void setup_outer_walls();
    void setup_inner_walls();
    void set_graphics_positions();
};

#endif // OBSTACLES_H
