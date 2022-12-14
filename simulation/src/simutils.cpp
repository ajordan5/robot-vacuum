#define _USE_MATH_DEFINES

#include "simutils.h"
#include "occupancygridutils.h"
#include <QtCore>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QPhongMaterial>

btRigidBody* local_create_rigidBody(btDynamicsWorld* m_ownerWorld, btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
    btVector3 localInertia(1, 1, 1);
    shape->calculateLocalInertia(mass, localInertia);

    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    body->setFriction(1.0);

    m_ownerWorld->addRigidBody(body);

    return body;
}

btVector3 rotate_ray_local(const btTransform& localFrame, const btVector3& ray, double angleDegrees)
{
    double angleRadians = deg_2_rad(angleDegrees);
    btVector3 rotated = ray.rotate({0,0,1}, angleRadians);
    return localFrame*rotated;
}

Qt3DCore::QEntity* create_cuboid_graphics(double xExtent, double yExtent, double zExtent, const QColor &bodyColor, Qt3DCore::QTransform* mTransform)
{
    Qt3DExtras::QCuboidMesh* bodyMesh = new Qt3DExtras::QCuboidMesh();
    bodyMesh->setXExtent(xExtent);
    bodyMesh->setYExtent(yExtent);
    bodyMesh->setZExtent(zExtent);

    Qt3DExtras::QPhongMaterial* material = new Qt3DExtras::QPhongMaterial();
    material->setDiffuse(bodyColor);

    Qt3DCore::QEntity* mEntity = new Qt3DCore::QEntity();
    mEntity->addComponent(bodyMesh);
    mEntity->addComponent(material);
    mEntity->addComponent(mTransform);
    mEntity->setEnabled(true);

    return mEntity;


}

Qt3DCore::QEntity* create_cylinder_graphics(double bodyRadius, double bodyThickness, const QColor &bodyColor, Qt3DCore::QTransform* mTransform)
{
    Qt3DExtras::QCylinderMesh* bodyMesh = new Qt3DExtras::QCylinderMesh();
    bodyMesh->setRadius(bodyRadius);
    bodyMesh->setLength(bodyThickness);

    Qt3DExtras::QPhongMaterial* material = new Qt3DExtras::QPhongMaterial();
    material->setDiffuse(bodyColor);

    Qt3DCore::QEntity* mEntity = new Qt3DCore::QEntity();
    mEntity->addComponent(bodyMesh);
    mEntity->addComponent(material);
    mEntity->addComponent(mTransform);
    mEntity->setEnabled(true);

    return mEntity;

}

double get_heading_of_z_rot_matrix(btTransform homogeneousTrans)
{
    btMatrix3x3 rotation{homogeneousTrans.getBasis()};
    double sinTh{rotation[1][0]};
    double cosTh{rotation[1][1]};
    return atan2(sinTh, cosTh);
}

bool is_arrowkey(int key)
{
    return (key == Qt::Key_Down || key == Qt::Key_Up || key == Qt::Key_Right || key == Qt::Key_Left);
}
