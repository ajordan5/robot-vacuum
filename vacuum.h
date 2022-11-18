#ifndef VACUUM_H
#define VACUUM_H

#include "btBulletDynamicsCommon.h"
#include <QMatrix4x4>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <QColor>

#include "bulletutils.h"

class Vacuum
{
public:
    Vacuum(btDynamicsWorld* ownerWorld, btVector3 initalPosition);
    btRigidBody** get_vehicle_body() {return m_bodies;}
    Qt3DCore::QEntity** getQEntity() {return mCylinderEntities;}
    void update_position();
    void drive(int key);
    void stop();

private:
    btDynamicsWorld* m_ownerWorld;
    btCollisionShape* m_shapes[3];
    btRigidBody* m_bodies[3];
    btHingeConstraint* m_joints[2];
    double bodyRadius{5};
    double halfBodyThickness{2};
    double wheelRadius{2.1};
    double halfWheelThickness{1};

    btScalar btMat[16];
    btTransform trans;

    QVector3D mResetPosition;
    Qt3DCore::QTransform *mTransforms[3];
    Qt3DCore::QEntity *mCylinderEntities[3];

    void setup_physics(const btVector3& initalPosition);
    void setup_graphics();


};


#endif // VACUUM_H
