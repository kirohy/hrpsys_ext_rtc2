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

CORBA::Boolean CollisionDetector2Service_impl::setCollisionDetector2Param(const hrpsys_ext_rtc::CollisionDetector2Service::CollisionDetector2Param& i_param)
{
  return this->m_collision->setCollisionDetector2Param(i_param);
};

CORBA::Boolean CollisionDetector2Service_impl::getCollisionDetector2Param(hrpsys_ext_rtc::CollisionDetector2Service::CollisionDetector2Param_out i_param)
{
  i_param = hrpsys_ext_rtc::CollisionDetector2Service::CollisionDetector2Param();
  return this->m_collision->getCollisionDetector2Param(i_param);
};

void CollisionDetector2Service_impl::collision(CollisionDetector2 *i_collision)
{
    m_collision = i_collision;
} 

