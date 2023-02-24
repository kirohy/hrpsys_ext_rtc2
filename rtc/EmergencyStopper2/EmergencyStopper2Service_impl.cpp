#include <iostream>
#include "EmergencyStopper2Service_impl.h"
#include "EmergencyStopper2.h"

void EmergencyStopper2Service_impl::stopMotion(const char *jname)
{
    comp_->stopMotion(jname);
}

void EmergencyStopper2Service_impl::releaseMotion(const char *jname)
{
    comp_->releaseMotion(jname);
}

void EmergencyStopper2Service_impl::startTorque(const char *jname)
{
    comp_->startTorque(jname);
}

CORBA::Boolean EmergencyStopper2Service_impl::setEmergencyStopper2Param(const hrpsys_ext_rtc::EmergencyStopper2Service::EmergencyStopper2Param& i_param)
{
  return this->comp_->setEmergencyStopper2Param(i_param);
};

CORBA::Boolean EmergencyStopper2Service_impl::getEmergencyStopper2Param(hrpsys_ext_rtc::EmergencyStopper2Service::EmergencyStopper2Param_out i_param)
{
  i_param = hrpsys_ext_rtc::EmergencyStopper2Service::EmergencyStopper2Param();
  return this->comp_->getEmergencyStopper2Param(i_param);
};
