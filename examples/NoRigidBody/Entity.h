#pragma once

#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "LinearMath/btVector3.h"

#include <memory>

class btCollisionWorld;
class btCollisionObject;

class Entity
{
public:
	template <typename Shape, typename... Args>
	static Entity* create(btCollisionWorld* world, Args... shapeArgs);

	Entity(btCollisionWorld* world, std::unique_ptr<btCollisionShape> shape);

	~Entity();

	void setName(std::string name) { m_name = std::move(name); }
	const std::string& getName() const { return m_name; }

	void setPosition(const btVector3& position);
	const btVector3& getPosition() const;

private:
	std::string m_name;

	btCollisionWorld* m_world{};
	btCollisionObject* m_collisionObject{};
	std::unique_ptr<btCollisionShape> m_shape;
};

template <typename Shape, typename... Args>
Entity* Entity::create(btCollisionWorld* world, Args... shapeArgs)
{
	return new Entity(world, std::unique_ptr<btCollisionShape>(new Shape(std::forward<Args>(shapeArgs)...)));
}

Entity* entityOf(const btCollisionObject* obj);
