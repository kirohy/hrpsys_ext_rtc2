// -*-C++-*-
#ifndef COLLISIONDETECTOR2SERVICE_IMPL_H
#define COLLISIONDETECTOR2SERVICE_IMPL_H

#include "hrpsys_ext_rtc/idl/CollisionDetector2Service.hh"

class CollisionDetector2;

class CollisionDetector2Service_impl
    : public virtual POA_hrpsys_ext_rtc::CollisionDetector2Service,
      public virtual PortableServer::RefCountServantBase
{
public:
    CollisionDetector2Service_impl();
    virtual ~CollisionDetector2Service_impl();
    //
    CORBA::Boolean enableCollisionDetection();
    CORBA::Boolean disableCollisionDetection();
    bool setCollisionDetector2Param(const hrpsys_ext_rtc::CollisionDetector2Service::CollisionDetector2Param& i_param);
    bool getCollisionDetector2Param(hrpsys_ext_rtc::CollisionDetector2Service::CollisionDetector2Param_out i_param);

    void collision(CollisionDetector2 *i_collision);
    //
private:
    CollisionDetector2 *m_collision;
};

#endif
