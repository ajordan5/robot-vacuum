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
    btCollisionShape* m_shapes[4];
    btRigidBody* m_bodies[4];
    double wallThickness{10};
    double wallHeight{50};

    btScalar btMat[16];
    btTransform trans;

    Qt3DCore::QTransform *mTransforms[4];
    Qt3DCore::QEntity *mEntities[4];

    void setup_physics();
    void setup_graphics();
    void set_graphics_positions();
};

#endif // OBSTACLES_H
