#include "Entity.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

Entity::Entity(btCollisionWorld* world, std::unique_ptr<btCollisionShape> shape)
	: m_world(world), m_shape(std::move(shape))
{
	btAssert(m_world);
	btAssert(m_shape);

	m_collisionObject = new btCollisionObject;
	m_collisionObject->setCollisionShape(m_shape.get());
	m_collisionObject->setCollisionFlags(btCollisionObject::CF_DYNAMIC_OBJECT);
	m_collisionObject->setUserPointer(this);

	m_world->addCollisionObject(m_collisionObject, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
}

Entity::~Entity()
{
	m_world->removeCollisionObject(m_collisionObject);

	m_collisionObject->setUserPointer(nullptr);
	delete m_collisionObject;
}

void Entity::setPosition(const btVector3& position)
{
	auto transform = m_collisionObject->getWorldTransform();
	transform.setOrigin(position);
	m_collisionObject->setWorldTransform(transform);
}

const btVector3& Entity::getPosition() const
{
	return m_collisionObject->getWorldTransform().getOrigin();
}

Entity* entityOf(const btCollisionObject* obj)
{
	return static_cast<Entity*>(obj->getUserPointer());
}
