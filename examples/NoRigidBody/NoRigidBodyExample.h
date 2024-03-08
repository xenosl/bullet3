#pragma once

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonCameraInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>

#include <vector>
#include <memory>

class Entity
{
public:
	template <typename Shape, typename... Args>
	static Entity* create(btCollisionWorld* world, Args... shapeArgs);

	Entity(btCollisionWorld* world, std::unique_ptr<btCollisionShape> shape);

	~Entity();

	void setPosition(const btVector3& position);
	[[nodiscard]] const btVector3& getPosition() const;

private:
	btCollisionWorld* m_world{};
	btCollisionObject* m_collisionObject{};
	std::unique_ptr<btCollisionShape> m_shape;
};

class EntityMover
{
public:
	explicit EntityMover(Entity* entity);

	void stepSimulation(float deltaTime);

	void setLinearMoveAxis(int axis) { m_linearMoveAxis = axis; }
	void setAngularMoveAxis(int axis) { m_angularMoveAxis = axis; }

	[[nodiscard]] Entity* entity() const { return m_entity; }

private:
	Entity* m_entity{};

	int m_linearMoveAxis = 0;
	int m_angularMoveAxis = 1;

	float m_moveTime = 0;

	float m_angularMoveSpeed = B3_2_PI;

	float m_linearMoveSpeed = 5;
	float m_linearMoveRangeMin = -10;
	float m_linearMoveRangeMax = 10;
};

class NoRigidBodyExample : public CommonExampleInterface
{
public:
	explicit NoRigidBodyExample(const CommonExampleOptions& options);

	void initPhysics() override;
	void exitPhysics() override;

private:
	CommonExampleOptions m_options;

	// Physics ---------------------------------------------------------------------------------------------------------
public:
	void initWorld();
	void exitWorld();
	void stepSimulation(float deltaTime) override;

private:
	void createDynamicsWorld();

	Entity* createEntity(double size, const btVector3& position, int linearMoveAxis, int angularMoveAxis);
	void createEntities();
	void destroyEntities();

	static void onContactStarted(btPersistentManifold* const& manifold);
	static void onContactEnded(btPersistentManifold* const& manifold);

	std::vector<Entity*> m_entities;
	std::vector<EntityMover*> m_entityMovers;

	btBroadphaseInterface* m_broadphase{};
	btCollisionDispatcher* m_dispatcher{};
	btConstraintSolver* m_solver{};
	btDefaultCollisionConfiguration* m_collisionConfiguration{};
	btDiscreteDynamicsWorld* m_dynamicsWorld{};

	// GUI -------------------------------------------------------------------------------------------------------------
public:
	[[nodiscard]] GUIHelperInterface* gui() const { return m_options.m_guiHelper; }

private:
	void initGui();
	void exitGui();

	// Rendering -------------------------------------------------------------------------------------------------------
public:
	void renderScene() override;
	void physicsDebugDraw(int debugFlags) override;

	void resetCamera() override;

private:
	void initRendering();
	void exitRendering();

	[[nodiscard]] btIDebugDraw* debugDrawer() const;

	// Input -----------------------------------------------------------------------------------------------------------
public:
	bool mouseMoveCallback(float x, float y) override;
	bool mouseButtonCallback(int button, int state, float x, float y) override;
	bool keyboardCallback(int key, int state) override;
};

template <typename Shape, typename... Args>
Entity* Entity::create(btCollisionWorld* world, Args... shapeArgs)
{
	return new Entity(world, std::unique_ptr<btCollisionShape>(new Shape(std::forward<Args>(shapeArgs)...)));
}

inline void Entity::setPosition(const btVector3& position)
{
	auto transform = m_collisionObject->getWorldTransform();
	transform.setOrigin(position);
	m_collisionObject->setWorldTransform(transform);
}

inline const btVector3& Entity::getPosition() const
{
	return m_collisionObject->getWorldTransform().getOrigin();
}

btIDebugDraw* NoRigidBodyExample::debugDrawer() const
{
	return m_dynamicsWorld ? m_dynamicsWorld->getDebugDrawer() : nullptr;
}
