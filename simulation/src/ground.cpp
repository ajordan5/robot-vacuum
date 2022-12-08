#include "ground.h"
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DCore/qtransform.h>

Ground::Ground()
{
  create_physics_representation();
}

Ground::~Ground()
{
    delete mRigidBody;
    delete mRigidCI;
    delete mGroundShape;
    delete mGroundMotionState;
}

Ground::Ground(QColor color)
{
  mColor=color;
  create_physics_representation();
  create_graphics_representation();
}

void Ground::create_physics_representation()
{
  mGroundShape = new btBoxShape(btVector3(mSize*.5,mSize*.5,mSize*.005));
  mGroundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,-mSize*.005)));
  mRigidCI= new btRigidBody::btRigidBodyConstructionInfo(0,mGroundMotionState,mGroundShape,btVector3(0,0,0));
  mRigidCI->m_restitution = 0.9;
  mRigidBody = new btRigidBody(*mRigidCI);
}

void Ground::create_graphics_representation()
{
    Qt3DExtras::QPlaneMesh *planeMesh = new Qt3DExtras::QPlaneMesh();
    planeMesh->setWidth(2);
    planeMesh->setHeight(2);
    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
    material->setDiffuse(mColor);

    Qt3DCore::QTransform *planeTransform = new Qt3DCore::QTransform();
    planeTransform->setScale(mSize);
    planeTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), 90.0f));
    planeTransform->setTranslation(QVector3D(0.0f, 0.0f, 0.0f));

    mPlaneEntity = new Qt3DCore::QEntity();
    mPlaneEntity->addComponent(planeMesh);
    mPlaneEntity->addComponent(material);
    mPlaneEntity->addComponent(planeTransform);
    mPlaneEntity->setEnabled(true);
}



