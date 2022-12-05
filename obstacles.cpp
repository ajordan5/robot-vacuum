#include "obstacles.h"

#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DExtras/QCuboidMesh>
#include <QColor>

Obstacles::Obstacles(btDynamicsWorld* ownerWorld)
    :  m_ownerWorld{ownerWorld }
{
    setup_physics();
    setup_graphics();
}

void Obstacles::setup_physics()
{
    btVector3 outerWallBody(wallThickness/2, outerWallLength/2, wallHeight/2);
    for (int i = 0; i < 4; ++i)
    {
        m_shapes[i] = new btBoxShape(outerWallBody);
    }

    outerWallBody = btVector3(btScalar(wallThickness/2), btScalar(outerWallLength/4), btScalar(wallHeight/2));
    m_shapes[4] = new btBoxShape(outerWallBody);


    btVector3 vRoot = btVector3(btScalar(0.), btScalar(outerWallLength/2), btScalar(0));
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(vRoot);
    m_bodies[0] = local_create_rigidBody(m_ownerWorld, btScalar(0.), transform, m_shapes[0]);

    vRoot = btVector3(btScalar(outerWallLength), btScalar(outerWallLength/2), btScalar(0));
    transform.setIdentity();
    transform.setOrigin(vRoot);
    m_bodies[1] = local_create_rigidBody(m_ownerWorld, btScalar(0.), transform, m_shapes[1]);

    vRoot = btVector3(btScalar(outerWallLength/2), btScalar(0), btScalar(0));
    transform.setIdentity();
    transform.setOrigin(vRoot);
    transform.setRotation(btQuaternion({0,0,1}, M_PI_2));
    m_bodies[2] = local_create_rigidBody(m_ownerWorld, btScalar(0.), transform, m_shapes[2]);

    vRoot = btVector3(btScalar(outerWallLength/2), btScalar(outerWallLength), btScalar(0));
    transform.setIdentity();
    transform.setOrigin(vRoot);
    transform.setRotation(btQuaternion({0,0,1}, M_PI_2));
    m_bodies[3] = local_create_rigidBody(m_ownerWorld, btScalar(0.), transform, m_shapes[3]);

    vRoot = btVector3(btScalar(outerWallLength/2), btScalar(outerWallLength/2), btScalar(0));
    transform.setIdentity();
    transform.setOrigin(vRoot);
    transform.setRotation(btQuaternion({0,0,1}, M_PI_2));
    m_bodies[4] = local_create_rigidBody(m_ownerWorld, btScalar(0.), transform, m_shapes[4]);
}

void Obstacles::setup_graphics()
{
    for (int i = 0; i < 4; ++i)
    {
        mTransforms[i] = new Qt3DCore::QTransform();
        mEntities[i] = create_cuboid(wallThickness, outerWallLength, wallHeight, Qt::blue, mTransforms[i]);
    }

    mTransforms[4] = new Qt3DCore::QTransform();
    mEntities[4] = create_cuboid(wallThickness, outerWallLength/2, wallHeight, Qt::blue, mTransforms[4]);

    set_graphics_positions();

}

void Obstacles::set_graphics_positions()
{
    for (int i = 0; i < 5; i++)
    {
        m_bodies[i]->getMotionState()->getWorldTransform(trans);
        trans.getOpenGLMatrix(btMat);

        QMatrix4x4 trans(btMat);
        trans=trans.transposed();
        mTransforms[i]->setMatrix(trans);
    }
}
