// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __SOFT_ERROR_LIMITER_SERVICE_H__
#define __SOFT_ERROR_LIMITER_SERVICE_H__

#include "hrpsys_ext_rtc/idl/SoftErrorLimiter2Service.hh"
#include <cnoid/Body>

class SoftErrorLimiter2;

class SoftErrorLimiter2Service_impl
	: public virtual POA_hrpsys_ext_rtc::SoftErrorLimiter2Service,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	SoftErrorLimiter2Service_impl();

	/**
	   \brief destructor
	*/
	virtual ~SoftErrorLimiter2Service_impl();

    void setServoErrorLimit(const char *jname, double limit);

    //
    void setComp(SoftErrorLimiter2* i_comp) {comp_ = i_comp; }

private:
	SoftErrorLimiter2 *comp_;
};

#endif
