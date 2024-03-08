#include "NoRigidBodyExample.h"
#include "NoRigidBodyExampleEntry.h"

#include <cmath>

Entity::Entity(btCollisionWorld* world, std::unique_ptr<btCollisionShape> shape)
	: m_world(world), m_shape(std::move(shape))
{
	btAssert(m_world);
	btAssert(m_shape);

	m_collisionObject = new btCollisionObject;
	m_collisionObject->setCollisionShape(m_shape.get());
	m_collisionObject->setCollisionFlags(btCollisionObject::CF_DYNAMIC_OBJECT);

	m_world->addCollisionObject(m_collisionObject, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
}

Entity::~Entity()
{
	m_world->removeCollisionObject(m_collisionObject);

	delete m_collisionObject;
}

EntityMover::EntityMover(Entity* entity)
	: m_entity(entity)
{
}

void EntityMover::stepSimulation(float deltaTime)
{
	m_moveTime += deltaTime;

	auto position = m_entity->getPosition();

	auto posAngular = std::sinf(m_moveTime * m_angularMoveSpeed);
	position[m_angularMoveAxis] = posAngular;

	auto moveDelta = m_linearMoveSpeed * deltaTime;
	auto posLinear = position[m_linearMoveAxis] + moveDelta;
	position[m_linearMoveAxis] = posLinear;

	if (position[m_linearMoveAxis] > m_linearMoveRangeMax)
	{
		position[m_linearMoveAxis] = m_linearMoveRangeMax;
		m_linearMoveSpeed = -m_linearMoveSpeed;
	}
	if (position[m_linearMoveAxis] < m_linearMoveRangeMin)
	{
		position[m_linearMoveAxis] = m_linearMoveRangeMin;
		m_linearMoveSpeed = -m_linearMoveSpeed;
	}

	m_entity->setPosition(position);
}

NoRigidBodyExample::NoRigidBodyExample(const CommonExampleOptions& options)
	: m_options(options)
{
}

void NoRigidBodyExample::initPhysics()
{
	createDynamicsWorld();
	createEntities();

	gContactAddedCallback = &NoRigidBodyExample::onContactAdded;

	auto gui = this->gui();
	gui->setUpAxis(1);
	gui->createPhysicsDebugDrawer(m_dynamicsWorld);
	gui->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void NoRigidBodyExample::exitPhysics()
{
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

void NoRigidBodyExample::createEntities()
{
	for (int i = 0; i < 2; ++i)
	{
		auto entity = Entity::create<btSphereShape>(m_dynamicsWorld, 0.5);
		//auto entity = Entity::create<btConeShapeX>(m_dynamicsWorld, 0.5, 1.0);
		//auto entity = Entity::create<btBoxShape>(m_dynamicsWorld, btVector3{0.1, 0.1, 0.1});
		m_entities.push_back(entity);
		entity->setPosition({0, 0, i * 2.0});

		auto mover = new EntityMover(entity);
		m_entityMovers.push_back(mover);
		mover->setLinearMoveAxis(i % 2);
		mover->setAngularMoveAxis(i % 2 + 1);
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

bool NoRigidBodyExample::onContactAdded(btManifoldPoint& cp,
	const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
	const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
{
	b3Printf("onContactAdded(%p, %p)", colObj0Wrap, colObj1Wrap);
	return true;
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
	float dist = 10;
	float pitch = -35;
	float yaw = 52;
	float targetPos[3] = {0, 0, 0};
	this->gui()->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
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
