#ifndef BT_CONTACT_CALLBACKS_H
#define BT_CONTACT_CALLBACKS_H

class btManifoldPoint;
class btPersistentManifold;

class btContactCallback
{
public:
	virtual ~btContactCallback() = default;

	virtual void onContactProcessed(btManifoldPoint& cp, void* body0, void* body1) = 0;
	virtual void onContactDestroyed(void* userPersistentData) = 0;

	virtual void onContactStarted(btPersistentManifold* const& manifold) = 0;
	virtual void onContactEnded(btPersistentManifold* const& manifold) = 0;
};

#endif  // BT_CONTACT_CALLBACKS_H
