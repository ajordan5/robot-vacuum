#include "robotvacuumsim.h"

RobotVacuumSim::RobotVacuumSim(struct GUIHelperInterface* helper)
    : m_guiHelper(helper),
          m_vacuumChassis(0),
          m_indexVertexArrays(0),
          m_vertices(0),
          m_cameraHeight(4.f),
          m_minCameraDistance(3.f),
          m_maxCameraDistance(10.f)
{
    helper->setUpAxis(1);
    m_vehicle = 0;
    m_wheelShape = 0;
//    m_cameraPosition = btVector3(30, 30, 30);
    m_useDefaultCamera = false;
}

RobotVacuumSim::~RobotVacuumSim()
{
    exitPhysics();
}

void RobotVacuumSim::exitPhysics()
{
    //cleanup in the reverse order of creation/initialization
}


void RobotVacuumSim::resetCamera()
{
    float dist = 8;
    float pitch = -32;
    float yaw = -45;
    float targetPos[3] = {-0.33, -0.72, 4.5};
    m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
}

void RobotVacuumSim::physicsDebugDraw(int debugFlags)
{
    if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
    {
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
        m_dynamicsWorld->debugDrawWorld();
    }
}

void RobotVacuumSim::initPhysics()
{


    setupSurroundings();
    setupSolver();
    setupVehicle();
    resetVehicle();

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void RobotVacuumSim::setupSurroundings()
{
    int upAxis = 1;

    m_guiHelper->setUpAxis(upAxis);

    btVector3 groundExtents(50, 50, 50);
    groundExtents[upAxis] = 3;
    btCollisionShape* groundShape = new btBoxShape(groundExtents);
    m_collisionShapes.push_back(groundShape);
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    btVector3 worldMin(-1000, -1000, -1000);
    btVector3 worldMax(1000, 1000, 1000);
    m_overlappingPairCache = new btAxisSweep3(worldMin, worldMax);

    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(0, -3, 0));
    localCreateRigidBody(0, tr, groundShape);
}

void RobotVacuumSim::setupSolver()
{
    btDantzigSolver* mlcp = new btDantzigSolver();
    //btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
    btMLCPSolver* sol = new btMLCPSolver(mlcp);
    m_constraintSolver = sol;

    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_constraintSolver, m_collisionConfiguration);
    m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;  //for direct solver it is better to have a small A matrix
    m_dynamicsWorld->getSolverInfo().m_globalCfm = 0.00001;

    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
}

void RobotVacuumSim::setupVehicle()
{
    btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
    m_collisionShapes.push_back(chassisShape);

    btCompoundShape* compound = new btCompoundShape();
    m_collisionShapes.push_back(compound);
    btTransform localTrans;
    localTrans.setIdentity();
    //localTrans effectively shifts the center of mass with respect to the chassis
    localTrans.setOrigin(btVector3(0, 1, 0));

    compound->addChildShape(localTrans, chassisShape);

    {
        btCollisionShape* suppShape = new btBoxShape(btVector3(0.5f, 0.1f, 0.5f));
        btTransform suppLocalTrans;
        suppLocalTrans.setIdentity();
        //localTrans effectively shifts the center of mass with respect to the chassis
        suppLocalTrans.setOrigin(btVector3(0, 1.0, 2.5));
        compound->addChildShape(suppLocalTrans, suppShape);
    }

    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(0, 0.f, 0));

    m_vacuumChassis = localCreateRigidBody(800, tr, compound);  //chassisShape);
    //m_carChassis->setDamping(0.2,0.2);

    m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));

    m_guiHelper->createCollisionShapeGraphicsObject(m_wheelShape);
    int wheelGraphicsIndex = m_wheelShape->getUserIndex();

    const float position[4] = {0, 10, 10};
    const float quaternion[4] = {0, 0, 0, 1};
    const float color[4] = {0, 1, 1};
    const float scaling[4] = {1, 1, 1};

    for (int i = 0; i < 3; i++)
    {
        m_wheelInstances[i] = m_guiHelper->registerGraphicsInstance(wheelGraphicsIndex, position, quaternion, color, scaling);
    }

    {
        m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
        m_vehicle = new btRaycastVehicle(m_tuning, m_vacuumChassis, m_vehicleRayCaster);

        m_vacuumChassis->setActivationState(DISABLE_DEACTIVATION);

        m_dynamicsWorld->addVehicle(m_vehicle);

        float connectionHeight = 1.2f;

        bool isFrontWheel = true;

        //choose coordinate system
        m_vehicle->setCoordinateSystem(0, 1, 2);

        btVector3 connectionPointCS0(CUBE_HALF_EXTENTS - (0.3 * wheelWidth), connectionHeight, 2 * CUBE_HALF_EXTENTS - wheelRadius);

        m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
        connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (0.3 * wheelWidth), connectionHeight, 2 * CUBE_HALF_EXTENTS - wheelRadius);

        m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
        connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (0.3 * wheelWidth), connectionHeight, -2 * CUBE_HALF_EXTENTS + wheelRadius);
        isFrontWheel = false;
        m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
        connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS - (0.3 * wheelWidth), connectionHeight, -2 * CUBE_HALF_EXTENTS + wheelRadius);
        m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

        for (int i = 0; i < m_vehicle->getNumWheels(); i++)
        {
            btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
            wheel.m_suspensionStiffness = suspensionStiffness;
            wheel.m_wheelsDampingRelaxation = suspensionDamping;
            wheel.m_wheelsDampingCompression = suspensionCompression;
            wheel.m_frictionSlip = wheelFriction;
            wheel.m_rollInfluence = rollInfluence;
        }
    }


}

btRigidBody* RobotVacuumSim::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        shape->calculateLocalInertia(mass, localInertia);

        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

    btRigidBody* body = new btRigidBody(cInfo);
    //body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
    btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
    body->setWorldTransform(startTransform);
#endif  //

    m_dynamicsWorld->addRigidBody(body);
    return body;
}

void RobotVacuumSim::renderScene()
{
    m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);

    for (int i = 0; i < m_vehicle->getNumWheels(); i++)
    {
        //synchronize the wheels with the (interpolated) chassis worldtransform
        m_vehicle->updateWheelTransform(i, true);

        CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
        if (renderer)
        {
            btTransform tr = m_vehicle->getWheelInfo(i).m_worldTransform;
            btVector3 pos = tr.getOrigin();
            btQuaternion orn = tr.getRotation();
            renderer->writeSingleInstanceTransformToCPU(pos, orn, m_wheelInstances[i]);
        }
    }

    m_guiHelper->render(m_dynamicsWorld);

    ATTRIBUTE_ALIGNED16(btScalar)
    m[16];
    int i;

    btVector3 wheelColor(1, 0, 0);

    btVector3 worldBoundsMin, worldBoundsMax;
    getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin, worldBoundsMax);

    for (i = 0; i < m_vehicle->getNumWheels(); i++)
    {
        //synchronize the wheels with the (interpolated) chassis worldtransform
        m_vehicle->updateWheelTransform(i, true);
        //draw wheels (cylinders)
        m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
        //		m_shapeDrawer->drawOpenGL(m,m_wheelShape,wheelColor,getDebugMode(),worldBoundsMin,worldBoundsMax);
    }
}

void RobotVacuumSim::stepSimulation(float deltaTime)
{
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    {
        int wheelIndex = 2;
        m_vehicle->applyEngineForce(gEngineForce, wheelIndex);
        m_vehicle->setBrake(gBreakingForce, wheelIndex);
        wheelIndex = 3;
        m_vehicle->applyEngineForce(gEngineForce, wheelIndex);
        m_vehicle->setBrake(gBreakingForce, wheelIndex);

        wheelIndex = 0;
        m_vehicle->setSteeringValue(gVehicleSteering, wheelIndex);
        wheelIndex = 1;
        m_vehicle->setSteeringValue(gVehicleSteering, wheelIndex);
    }

    float dt = deltaTime;

    if (m_dynamicsWorld)
    {
        if (m_dynamicsWorld->getConstraintSolver()->getSolverType() == BT_MLCP_SOLVER)
        {
            btMLCPSolver* sol = (btMLCPSolver*)m_dynamicsWorld->getConstraintSolver();
            int numFallbacks = sol->getNumFallbacks();
            if (numFallbacks)
            {
                static int totalFailures = 0;
                totalFailures += numFallbacks;
                printf("MLCP solver failed %d times, falling back to btSequentialImpulseSolver (SI)\n", totalFailures);
            }
            sol->setNumFallbacks(0);
        }

    }

}

void RobotVacuumSim::clientResetScene()
{
    exitPhysics();
    initPhysics();
}

void RobotVacuumSim::resetVehicle()
{
    gVehicleSteering = 0.f;
    gBreakingForce = defaultBreakingForce;
    gEngineForce = 0.f;

    m_vacuumChassis->setCenterOfMassTransform(btTransform::getIdentity());
    m_vacuumChassis->setLinearVelocity(btVector3(0, 0, 0));
    m_vacuumChassis->setAngularVelocity(btVector3(0, 0, 0));
    m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_vacuumChassis->getBroadphaseHandle(), getDynamicsWorld()->getDispatcher());
    if (m_vehicle)
    {
        m_vehicle->resetSuspension();
        for (int i = 0; i < m_vehicle->getNumWheels(); i++)
        {
            //synchronize the wheels with the (interpolated) chassis worldtransform
            m_vehicle->updateWheelTransform(i, true);
        }
    }

}

bool RobotVacuumSim::keyboardCallback(int key, int state)
{
    bool handled = false;

    if (state)
    {


        switch (key)
        {
            case B3G_LEFT_ARROW:
            {
                handled = true;
                gVehicleSteering += steeringIncrement;
                if (gVehicleSteering > steeringClamp)
                    gVehicleSteering = steeringClamp;

                break;
            }
            case B3G_RIGHT_ARROW:
            {
                handled = true;
                gVehicleSteering -= steeringIncrement;
                if (gVehicleSteering < -steeringClamp)
                    gVehicleSteering = -steeringClamp;

                break;
            }
            case B3G_UP_ARROW:
            {
                handled = true;
                gEngineForce = maxEngineForce;
                gBreakingForce = 0.f;
                break;
            }
            case B3G_DOWN_ARROW:
            {
                handled = true;
                gEngineForce = -maxEngineForce;
                gBreakingForce = 0.f;
                break;
            }
        default:
            break;

        }
    }

    return handled;

}


CommonExampleInterface* RobotVacuumSimCreateFunc(struct CommonExampleOptions& options)
{
    return new RobotVacuumSim(options.m_guiHelper);
}
