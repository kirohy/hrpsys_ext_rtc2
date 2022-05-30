#include "CollisionDetector2Service_impl.h"
#include "CollisionDetector2.h"

CollisionDetector2Service_impl::CollisionDetector2Service_impl() : m_collision(NULL)
{
}

CollisionDetector2Service_impl::~CollisionDetector2Service_impl()
{
}

CORBA::Boolean CollisionDetector2Service_impl::enableCollisionDetection()
{
    return m_collision->enable();
}

CORBA::Boolean CollisionDetector2Service_impl::disableCollisionDetection()
{
    return m_collision->disable();
}

CORBA::Boolean CollisionDetector2Service_impl::setTolerance(const char *i_link_pair_name, CORBA::Double d_tolerance)
{
    return m_collision->setTolerance(i_link_pair_name, d_tolerance);
}

CORBA::Boolean CollisionDetector2Service_impl::setCollisionLoop(CORBA::Short loop)
{
	return m_collision->setCollisionLoop(loop);
}

CORBA::Boolean CollisionDetector2Service_impl::getCollisionStatus(OpenHRP::CollisionDetectorService::CollisionState_out state)
{
    state = new OpenHRP::CollisionDetectorService::CollisionState;
    return m_collision->getCollisionStatus(*state);
}

void CollisionDetector2Service_impl::collision(CollisionDetector2 *i_collision)
{
    m_collision = i_collision;
} 

