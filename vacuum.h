#ifndef VACUUM_H
#define VACUUM_H

#include "btBulletDynamicsCommon.h"
#include <QMatrix4x4>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <QColor>

#include "simutils.h"
#include "lidar.h"
#include "lidarmapper.h"

class Vacuum
{
public:
    Vacuum(btDynamicsWorld* ownerWorld, btVector3 initalPosition);
    btRigidBody** get_vehicle_body() {return m_bodies;}
    Qt3DCore::QEntity** getQEntity() {return mCylinderEntities;}
    double get_lidar_range() {return lidarRange;}
    void update_position();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> update_measurements();
    double get_heading();
    VehicleState get_state();
    void drive(int key);
    void stop();    

private:
    btDynamicsWorld* m_ownerWorld;
    btCollisionShape* m_shapes[4];
    btRigidBody* m_bodies[4];
    btHingeConstraint* m_joints[2];
    double bodyRadius{5};
    double halfBodyThickness{2};
    double wheelRadius{2.4};
    double halfWheelThickness{0.5};
    double lidarRange{40};
    Lidar* lidar;
    VehicleState state;

    btScalar btMat[16];
    btTransform trans;

    QVector3D mResetPosition;
    Qt3DCore::QTransform *mTransforms[4];
    Qt3DCore::QEntity *mCylinderEntities[4];

    void setup_physics(const btVector3& initalPosition);
    void setup_graphics();


};


#endif // VACUUM_H
