#ifndef VACUUM_H
#define VACUUM_H

#include "btBulletDynamicsCommon.h"
#include <QMatrix4x4>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <QColor>

class Vacuum
{
public:
    Vacuum(btDynamicsWorld* ownerWorld, btVector3 initalPosition);
    btRigidBody* get_vehicle_body() {return m_bodies[0];}

private:
    btDynamicsWorld* m_ownerWorld;
    btCollisionShape* m_shapes[3];
    btRigidBody* m_bodies[3];
    btTypedConstraint* m_joints[2];
    double bodyRadius{5};
    double halfBodyThickness{2};
    double wheelRadius{2};
    double halfWheelThickness{1};
    QVector3D mResetPosition;

    Qt3DCore::QTransform *mTransforms[3];
    Qt3DCore::QEntity *mCylinderEntities[3];

    void setup_physics(const btVector3& initalPosition);
    void setup_graphics();
    btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);

};

#endif // VACUUM_H
