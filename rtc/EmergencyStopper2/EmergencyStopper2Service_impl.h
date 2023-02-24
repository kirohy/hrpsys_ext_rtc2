#ifndef __EMERGENCY_STOPPER2_SERVICE_H__
#define __EMERGENCY_STOPPER2_SERVICE_H__

#include "hrpsys_ext_rtc/idl/EmergencyStopper2Service.hh"

class EmergencyStopper2;

class EmergencyStopper2Service_impl
    : public virtual POA_hrpsys_ext_rtc::EmergencyStopper2Service,
      public virtual PortableServer::RefCountServantBase
{
public:
    void stopMotion(const char *jname);
    void releaseMotion(const char *jname);
    void startTorque(const char *jname);

    bool setEmergencyStopper2Param(const hrpsys_ext_rtc::EmergencyStopper2Service::EmergencyStopper2Param& i_param);
    bool getEmergencyStopper2Param(hrpsys_ext_rtc::EmergencyStopper2Service::EmergencyStopper2Param_out i_param);

    //
    void setComp(EmergencyStopper2* i_comp) {comp_ = i_comp; }

private:
    EmergencyStopper2 *comp_;
};

#endif
