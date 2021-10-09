#include <iostream>
#include "SoftErrorLimiter2Service_impl.h"
#include "SoftErrorLimiter2.h"

SoftErrorLimiter2Service_impl::SoftErrorLimiter2Service_impl()
{
}

SoftErrorLimiter2Service_impl::~SoftErrorLimiter2Service_impl()
{
}

void SoftErrorLimiter2Service_impl::setServoErrorLimit(const char *jname, double limit)
{
    comp_->setServoErrorLimit(jname, limit);
}

