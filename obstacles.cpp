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
    setup_outer_walls();
    setup_inner_walls();
}

void Obstacles::setup_inner_walls()
{
    btVector3 wallBody = btVector3(btScalar(wallThickness/2), btScalar(outerWallLength/4), btScalar(wallHeight/2));
    m_shapes[4] = new btBoxShape(wallBody);

    wallBody = btVector3(btScalar(wallThickness/2), btScalar(outerWallLength/8), btScalar(wallHeight/2));
    m_shapes[5] = new btBoxShape(wallBody);
    m_shapes[6] = new btBoxShape(wallBody);

    wallBody = btVector3(btScalar(wallHeight), btScalar(wallHeight), btScalar(wallHeight/2));
    m_shapes[7] = new btBoxShape(wallBody);

    btTransform transform;
    btVector3 vRoot = btVector3(btScalar(outerWallLength/2), btScalar(outerWallLength/4), btScalar(0));
    transform.setIdentity();
    transform.setOrigin(vRoot);
    transform.setRotation(btQuaternion({0,0,1}, M_PI_2));
    m_bodies[4] = local_create_rigidBody(m_ownerWorld, btScalar(0.), transform, m_shapes[4]);

    vRoot = btVector3(btScalar(outerWallLength/2), btScalar(outerWallLength/8), btScalar(0));
    transform.setIdentity();
    transform.setOrigin(vRoot);
    m_bodies[5] = local_create_rigidBody(m_ownerWorld, btScalar(0.), transform, m_shapes[5]);

    vRoot = btVector3(btScalar(330), btScalar(330), btScalar(0));
    transform.setIdentity();
    transform.setOrigin(vRoot);
    transform.setRotation(btQuaternion({0,0,1}, M_PI_4));
    m_bodies[6] = local_create_rigidBody(m_ownerWorld, btScalar(0.), transform, m_shapes[6]);

    vRoot = btVector3(btScalar(outerWallLength/4), btScalar(3*outerWallLength/4), btScalar(0));
    transform.setIdentity();
    transform.setOrigin(vRoot);
    m_bodies[7] = local_create_rigidBody(m_ownerWorld, btScalar(0.), transform, m_shapes[7]);
}

void Obstacles::setup_outer_walls()
{
    btVector3 outerWallBody(wallThickness/2, outerWallLength/2, wallHeight/2);
    for (int i = 0; i < 4; ++i)
    {
        m_shapes[i] = new btBoxShape(outerWallBody);
    }

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
}

void Obstacles::setup_graphics()
{
    for (int i = 0; i < 4; ++i)
    {
        mTransforms[i] = new Qt3DCore::QTransform();
        mEntities[i] = create_cuboid_graphics(wallThickness, outerWallLength, wallHeight, Qt::blue, mTransforms[i]);
    }

    mTransforms[4] = new Qt3DCore::QTransform();
    mEntities[4] = create_cuboid_graphics(wallThickness, outerWallLength/2, wallHeight, Qt::blue, mTransforms[4]);

    mTransforms[5] = new Qt3DCore::QTransform();
    mEntities[5] = create_cuboid_graphics(wallThickness, outerWallLength/4, wallHeight, Qt::blue, mTransforms[5]);

    mTransforms[6] = new Qt3DCore::QTransform();
    mEntities[6] = create_cuboid_graphics(wallThickness, outerWallLength/4, wallHeight, Qt::blue, mTransforms[6]);

    mTransforms[7] = new Qt3DCore::QTransform();
    mEntities[7] = create_cuboid_graphics(wallHeight*2, wallHeight*2, wallHeight, Qt::blue, mTransforms[7]);

    set_graphics_positions();

}

void Obstacles::set_graphics_positions()
{
    for (int i = 0; i < 8; i++)
    {
        m_bodies[i]->getMotionState()->getWorldTransform(trans);
        trans.getOpenGLMatrix(btMat);

        QMatrix4x4 trans(btMat);
        trans=trans.transposed();
        mTransforms[i]->setMatrix(trans);
    }
}

void Obstacles::set_graphics_parent(Qt3DCore::QEntity* mRootEntity)
{
    for(Qt3DCore::QEntity* entity : mEntities)
    {
        entity->setParent(mRootEntity);
    }
}
