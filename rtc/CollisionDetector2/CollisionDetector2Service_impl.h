// -*-C++-*-
#ifndef COLLISIONDETECTOR2SERVICE_IMPL_H
#define COLLISIONDETECTOR2SERVICE_IMPL_H

#include "hrpsys/idl/CollisionDetectorService.hh"

using namespace OpenHRP;

class CollisionDetector2;

class CollisionDetector2Service_impl 
    : public virtual POA_OpenHRP::CollisionDetectorService,
      public virtual PortableServer::RefCountServantBase
{
public:
    CollisionDetector2Service_impl();
    virtual ~CollisionDetector2Service_impl();
    //
    CORBA::Boolean enableCollisionDetection();
    CORBA::Boolean disableCollisionDetection();
    CORBA::Boolean setTolerance(const char *i_link_pair_name, CORBA::Double d_tolerance);
    CORBA::Boolean setCollisionLoop(CORBA::Short loop);
    CORBA::Boolean getCollisionStatus(OpenHRP::CollisionDetectorService::CollisionState_out state);
    void collision(CollisionDetector2 *i_collision);
    //
private:
    CollisionDetector2 *m_collision;
};

#endif
