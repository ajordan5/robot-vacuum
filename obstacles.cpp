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

    btVector3 vRoot = btVector3(btScalar(0.), btScalar(0), btScalar(0));
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(vRoot);
    m_bodies[0] = local_create_rigidBody(m_ownerWorld, btScalar(0.), transform, m_shapes[0]);


}

void Obstacles::setup_graphics()
{
    Qt3DExtras::QCuboidMesh* bodyMesh = new Qt3DExtras::QCuboidMesh();
    // TODO put creating these shapes in a util function that takes sizes, rotation and shape
    bodyMesh->setXExtent(wallThickness);
    bodyMesh->setYExtent(200);
    bodyMesh->setZExtent(wallHeight*2);
    mTransforms[0] = new Qt3DCore::QTransform();
//    mTransforms[0]->setTranslation(mResetPosition);

    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
    material->setDiffuse(Qt::blue);

    mEntities[0] = new Qt3DCore::QEntity();
    mEntities[0]->addComponent(bodyMesh);
    mEntities[0]->addComponent(material);
    mEntities[0]->addComponent(mTransforms[0]);
    mEntities[0]->setEnabled(true);

    set_graphics_positions();

}

void Obstacles::set_graphics_positions()
{
    for (int i = 0; i < 1; i++)
    {
        m_bodies[i]->getMotionState()->getWorldTransform(trans);
        trans.getOpenGLMatrix(btMat);

        QMatrix4x4 trans(btMat);
        trans=trans.transposed();//Converting between opengl column major to Qt row major
        mTransforms[i]->setMatrix(trans);
    }
}
