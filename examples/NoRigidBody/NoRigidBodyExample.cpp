#include "NoRigidBodyExample.h"
#include "NoRigidBodyExampleEntry.h"
#include "Entity.h"
#include "EntityMover.h"

#include <cmath>

NoRigidBodyExample::NoRigidBodyExample(const CommonExampleOptions& options)
	: m_options(options)
{
}

void NoRigidBodyExample::initPhysics()
{
	initWorld();
	initGui();
	initRendering();
}

void NoRigidBodyExample::exitPhysics()
{
	exitRendering();
	exitGui();
	exitWorld();
}

void NoRigidBodyExample::initWorld()
{
	createDynamicsWorld();
	createEntitiesWithSinMover();

	m_contactCallback = new ContactCallback;
	m_dispatcher->contactCallback = m_contactCallback;
	//gContactAddedCallback = &NoRigidBodyExample::onContactAdded;
}

void NoRigidBodyExample::exitWorld()
{
	m_dispatcher->contactCallback = nullptr;
	delete m_contactCallback;
	m_contactCallback = nullptr;

	destroyEntities();

	if (m_dynamicsWorld)
	{
		for (int i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
			m_dynamicsWorld->removeConstraint(m_dynamicsWorld->getConstraint(i));

		for (int i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
		{
			btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
			{
				delete body->getMotionState();
			}
			m_dynamicsWorld->removeCollisionObject(obj);
			delete obj;
		}
	}

	delete m_dynamicsWorld;
	m_dynamicsWorld = nullptr;

	delete m_solver;
	m_solver = nullptr;

	delete m_broadphase;
	m_broadphase = nullptr;

	delete m_dispatcher;
	m_dispatcher = nullptr;

	delete m_collisionConfiguration;
	m_collisionConfiguration = nullptr;
}

void NoRigidBodyExample::stepSimulation(float deltaTime)
{
	for (auto em : m_entityMovers)
		em->stepSimulation(deltaTime);

	if (m_dynamicsWorld)
		m_dynamicsWorld->stepSimulation(deltaTime);
}

void NoRigidBodyExample::ContactCallback::onContactProcessed(btManifoldPoint& cp, void* body0, void* body1)
{
}

void NoRigidBodyExample::ContactCallback::onContactDestroyed(void* userPersistentData)
{
}

void NoRigidBodyExample::ContactCallback::onContactStarted(btPersistentManifold* const& manifold)
{
	b3Printf("onContactStarted(%p, %p)", manifold->getBody0(), manifold->getBody1());
}

void NoRigidBodyExample::ContactCallback::onContactEnded(btPersistentManifold* const& manifold)
{
	b3Printf("onContactEnded(%p, %p)", manifold->getBody0(), manifold->getBody1());
}

void NoRigidBodyExample::createDynamicsWorld()
{
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));

	auto debugDrawer = this->debugDrawer();
	if (debugDrawer)
		debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
}

Entity* NoRigidBodyExample::createEntityWithSinMover(double size, const btVector3& position, int linearMoveAxis, int angularMoveAxis)
{
	auto entity = Entity::create<btSphereShape>(m_dynamicsWorld, size / 2.0);
	//auto entity = Entity::create<btConeShapeX>(m_dynamicsWorld, 0.5, 1.0);
	//auto entity = Entity::create<btBoxShape>(m_dynamicsWorld, btVector3{0.1, 0.1, 0.1});
	m_entities.push_back(entity);
	entity->setPosition(position);

	auto mover = new EntitySinMover(entity);
	m_entityMovers.push_back(mover);
	mover->setLinearMoveAxis(linearMoveAxis);
	mover->setAngularMoveAxis(angularMoveAxis);
	//mover->setAngularMoveEnabled(false);

	return entity;
}

void NoRigidBodyExample::createEntitiesWithSinMover()
{
	static constexpr int EntityCountPerAxis = 100;
	static constexpr double EntitySize = 0.5;

	double gap = 1.0;
	double startPosition = -EntityCountPerAxis * gap / 2.0;
	for (int i = 0; i < EntityCountPerAxis; ++i)
	{
		createEntityWithSinMover(EntitySize, {0, 0, startPosition + i * gap}, 0, 1);
		createEntityWithSinMover(EntitySize, {0, 0, startPosition + i * gap}, 1, 0);
	}
}

void NoRigidBodyExample::destroyEntities()
{
	for (int i = static_cast<int>(m_entities.size()) - 1; i >= 0; --i)
	{
		auto mover = m_entityMovers.at(i);
		m_entityMovers.erase(m_entityMovers.begin() + i);
		delete mover;

		auto entity = m_entities.at(i);
		m_entities.erase(m_entities.begin() + i);
		delete entity;
	}
}

void NoRigidBodyExample::initGui()
{
	auto gui = this->gui();
	gui->setUpAxis(1);
	gui->createPhysicsDebugDrawer(m_dynamicsWorld);
	gui->autogenerateGraphicsObjects(m_dynamicsWorld);

	//	ButtonParams button("Show FPS", 0, true);
	//	bool* ptr = &gDisplayProfileInfo;
	//	button.m_initialState = *ptr;
	//	button.m_userPointer = ptr;
	//	button.m_callback = boolPtrButtonCallback;
	//	m_guiHelper->getParameterInterface()->registerButtonParameter(button);
}

void NoRigidBodyExample::exitGui()
{
}

void NoRigidBodyExample::renderScene()
{
	if (!m_dynamicsWorld)
		return;

	auto gui = this->gui();
	gui->syncPhysicsToGraphics(m_dynamicsWorld);
	gui->render(m_dynamicsWorld);
}

void NoRigidBodyExample::physicsDebugDraw(int debugFlags)
{
	if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
	{
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
		m_dynamicsWorld->debugDrawWorld();
	}
}

void NoRigidBodyExample::resetCamera()
{
	float dist = 25;
	float pitch = -35;
	float yaw = 52;
	float targetPos[3] = {0, 0, 0};
	this->gui()->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
}

void NoRigidBodyExample::initRendering()
{
}

void NoRigidBodyExample::exitRendering()
{
}

bool NoRigidBodyExample::mouseMoveCallback(float x, float y)
{
	return false;
}

bool NoRigidBodyExample::mouseButtonCallback(int button, int state, float x, float y)
{
	return false;
}

bool NoRigidBodyExample::keyboardCallback(int key, int state)
{
	return false;
}

CommonExampleInterface* NoRigidBodyExampleCreateFunc(CommonExampleOptions& options)
{
	return new NoRigidBodyExample(options);
}
