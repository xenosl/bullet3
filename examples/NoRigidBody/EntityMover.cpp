#include "EntityMover.h"
#include "Entity.h"

#include <cmath>

EntityMover::EntityMover(Entity* entity)
	: m_entity(entity)
{
}

EntitySinMover::EntitySinMover(Entity* entity)
	: EntityMover(entity)
{
}

void EntitySinMover::stepSimulation(float deltaTime)
{
	auto entity = this->entity();

	m_moveTime += deltaTime;

	auto position = entity->getPosition();

	if (m_angularMoveEnabled)
	{
		auto posAngular = std::sinf(m_moveTime * m_angularMoveSpeed);
		position[m_angularMoveAxis] = posAngular;
	}

	if (m_linearMoveEnabled)
	{
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
	}

	entity->setPosition(position);
}
