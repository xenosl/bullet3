#pragma once

#include "Bullet3Common/b3Scalar.h"

class Entity;

class EntityMover
{
public:
	explicit EntityMover(Entity* entity);
	virtual ~EntityMover() = default;

	virtual void stepSimulation(float deltaTime) = 0;

	[[nodiscard]] Entity* entity() const { return m_entity; }

private:
	Entity* m_entity{};
};

class EntitySinMover : public EntityMover
{
public:
	explicit EntitySinMover(Entity* entity);

	void stepSimulation(float deltaTime) override;

	void setLinearMoveAxis(int axis) { m_linearMoveAxis = axis; }
	void setAngularMoveAxis(int axis) { m_angularMoveAxis = axis; }

	void setAngularMoveEnabled(bool enabled) { m_angularMoveEnabled = enabled; }
	void setLinearMoveEnabled(bool enabled) { m_linearMoveEnabled = enabled; }

private:
	int m_linearMoveAxis = 0;
	int m_angularMoveAxis = 1;

	float m_moveTime = 0;

	bool m_angularMoveEnabled = true;
	float m_angularMoveSpeed = B3_2_PI;

	bool m_linearMoveEnabled = true;
	float m_linearMoveSpeed = 5;
	float m_linearMoveRangeMin = -10;
	float m_linearMoveRangeMax = 10;
};
