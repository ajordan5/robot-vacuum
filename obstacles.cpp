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
    btVector3 wallBody(wallThickness/2, 100, wallHeight);
    m_shapes[0] = new btBoxShape(wallBody);

    btVector3 vRoot = btVector3(btScalar(50.), btScalar(100), btScalar(0));
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(vRoot);
    m_bodies[0] = local_create_rigidBody(m_ownerWorld, btScalar(0.), transform, m_shapes[0]);

    m_shapes[1] = new btBoxShape(wallBody);

    vRoot = btVector3(btScalar(150.), btScalar(100), btScalar(0));
    transform.setIdentity();
    transform.setOrigin(vRoot);
    m_bodies[1] = local_create_rigidBody(m_ownerWorld, btScalar(0.), transform, m_shapes[0]);
}

void Obstacles::setup_graphics()
{
    for (int i = 0; i < 2; ++i)
    {
        mTransforms[i] = new Qt3DCore::QTransform();
        mEntities[i] = create_cuboid(wallThickness, 200, wallHeight, Qt::blue, mTransforms[i]);
    }

    set_graphics_positions();

}

void Obstacles::set_graphics_positions()
{
    for (int i = 0; i < 2; i++)
    {
        m_bodies[i]->getMotionState()->getWorldTransform(trans);
        trans.getOpenGLMatrix(btMat);

        QMatrix4x4 trans(btMat);
        trans=trans.transposed();
        mTransforms[i]->setMatrix(trans);
    }
}
