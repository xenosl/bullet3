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

class Entity;
class EntityMover;

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
	class ContactCallback : public btContactCallback
	{
	public:
		explicit ContactCallback(NoRigidBodyExample* owner);

		void onContactProcessed(btManifoldPoint& cp, void* body0, void* body1) override;
		void onContactDestroyed(void* userPersistentData) override;

		void onContactStarted(btPersistentManifold* const& manifold) override;
		void onContactEnded(btPersistentManifold* const& manifold) override;

	private:
		NoRigidBodyExample* m_owner;
	};

	void createDynamicsWorld();

	std::pair<Entity*, EntityMover*> createEntityWithSinMover(
		std::string name, double size, const btVector3& position, int linearMoveAxis, int angularMoveAxis);
	void createEntitiesWithSinMover();

	void destroyEntities();

	std::vector<Entity*> m_entities;
	std::vector<EntityMover*> m_entityMovers;

	btBroadphaseInterface* m_broadphase{};
	btCollisionDispatcher* m_dispatcher{};
	btConstraintSolver* m_solver{};
	btDefaultCollisionConfiguration* m_collisionConfiguration{};
	btDiscreteDynamicsWorld* m_dynamicsWorld{};

	ContactCallback* m_contactCallback{};

	int m_simulatedStepCount = 0;

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

btIDebugDraw* NoRigidBodyExample::debugDrawer() const
{
	return m_dynamicsWorld ? m_dynamicsWorld->getDebugDrawer() : nullptr;
}
