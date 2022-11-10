#ifndef ROBOTVACUUMSIM_H
#define ROBOTVACUUMSIM_H

#define CUBE_HALF_EXTENTS 1

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "C:/bullet3/examples/CommonInterfaces/CommonExampleInterface.h"
//#include "CommonInterfaces/CommonExampleInterface.h"
#include "C:/bullet3/examples/CommonInterfaces/CommonRenderInterface.h"
#include "C:/bullet3/examples/CommonInterfaces/CommonWindowInterface.h"
#include "C:/bullet3/examples/CommonInterfaces/CommonGUIHelperInterface.h"

class RobotVacuumSim : public CommonExampleInterface
{
public:
    GUIHelperInterface* m_guiHelper;

    RobotVacuumSim(struct GUIHelperInterface* helper);
    ~RobotVacuumSim();

    btDefaultCollisionConfiguration* collisionConfiguration;
    btDiscreteDynamicsWorld* m_dynamicsWorld;
    btTransform startTransform;

    btDiscreteDynamicsWorld* getDynamicsWorld();

    btRigidBody* m_vacuumChassis;
    btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

    int m_wheelInstances[3];
    float wheelRadius{0.5f};
    float wheelWidth{0.4f};
    btVector3 wheelDirectionCS0{0, -1, 0};
    btVector3 wheelAxleCS{-1, 0, 0};
    float wheelFriction{1000};  //BT_LARGE_FLOAT;
    float suspensionStiffness{20.f};
    float suspensionDamping{2.3f};
    btScalar suspensionRestLength{0.6};
    float suspensionCompression{4.4f};
    float rollInfluence{0.1f};

    bool m_useDefaultCamera;

    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

    class btBroadphaseInterface* m_overlappingPairCache;

    class btCollisionDispatcher* m_dispatcher;

    class btConstraintSolver* m_constraintSolver;

    class btDefaultCollisionConfiguration* m_collisionConfiguration;

    class btTriangleIndexVertexArray* m_indexVertexArrays;

    btVector3* m_vertices;

    btRaycastVehicle::btVehicleTuning m_tuning;
    btVehicleRaycaster* m_vehicleRayCaster;
    btRaycastVehicle* m_vehicle;
    btCollisionShape* m_wheelShape;

    float m_cameraHeight;
    float m_minCameraDistance;
    float m_maxCameraDistance;

    virtual void stepSimulation(float deltaTime);
    float gEngineForce{0.f};
    float defaultBreakingForce{10.f};
    float gBreakingForce{100.f};
    float maxEngineForce{1000.f};
    float maxBreakingForce{100.f};
    float gVehicleSteering{0.f};
    float steeringIncrement{0.04f};
    float steeringClamp{0.3f};


    virtual void resetVehicle();

    virtual void clientResetScene();

    virtual bool mouseMoveCallback(float x, float y)
    {
        return false;
    }

    virtual bool mouseButtonCallback(int button, int state, float x, float y)
    {
        return false;
    }

    virtual bool keyboardCallback(int key, int state);

    virtual void renderScene();

    virtual void physicsDebugDraw(int debugFlags);

    void initPhysics();
    void exitPhysics();
    void setupSurroundings();
    void setupSolver();
    void setupVehicle();

    virtual void resetCamera();



};

class CommonExampleInterface* RobotVacuumSimCreateFunc(struct CommonExampleOptions& options);


#endif // ROBOTVACUUMSIM_H
