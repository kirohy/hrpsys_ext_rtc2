#ifndef EMERGENCY_STOPPER2_H
#define EMERGENCY_STOPPER2_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cnoid/Body>
#include "EmergencyStopper2Service_impl.h"
#include <robot_hardware/idl/RobotHardware2Service.hh>
#include <cpp_filters/TwoPointInterpolator.h>
#include <mutex>

class EmergencyStopper2 : public RTC::DataFlowComponentBase
{
 public:
  EmergencyStopper2(RTC::Manager* manager);

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool stopMotion(const char *jname);
  bool releaseMotion(const char *jname);
  bool startTorque(const char *jname);
  bool setEmergencyStopper2Param(const hrpsys_ext_rtc::EmergencyStopper2Service::EmergencyStopper2Param& i_param);
  bool getEmergencyStopper2Param(hrpsys_ext_rtc::EmergencyStopper2Service::EmergencyStopper2Param& i_param);

 protected:

  RTC::TimedDoubleSeq m_qRef;
  RTC::TimedDoubleSeq m_tauRef;
  RTC::TimedDoubleSeq m_qAct;
  RTC::TimedDoubleSeq m_tauCtl;
  RTC::TimedLong m_stopSignal;
  RTC::TimedLong m_releaseSignal;
  RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_tauRefIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_qActIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_tauCtlIn;
  RTC::InPort<RTC::TimedLong> m_stopSignalIn;
  RTC::InPort<RTC::TimedLong> m_releaseSignalIn;

  RTC::TimedDoubleSeq m_q;
  RTC::TimedDoubleSeq m_tau;
  RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;
  RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut;

  RTC::CorbaPort m_EmergencyStopper2ServicePort;
  EmergencyStopper2Service_impl m_service0;

  RTC::CorbaPort m_RobotHardwareServicePort;
  RTC::CorbaConsumer<robot_hardware::RobotHardware2Service> m_robotHardwareService0;

 private:
  std::mutex mutex_;

  class Joint{
  public:
    enum Mode_enum{ MODE_REF, MODE_STOP, MODE_TORQUE};
    Mode_enum mode = MODE_REF;
    double interpolationTime = 0.0;
    bool changeGain = false;
    double qRef = 0.0;
    double tauRef = 0.0;
    double qAct = 0.0;
    double tauCtl = 0.0;
    cpp_filters::TwoPointInterpolator<double> qOut{0.0, 0.0, 0.0, cpp_filters::HOFFARBIB};
    double tauOut = 0.0;
  public:
    void init(){
      mode = MODE_REF;
      interpolationTime = 0.0;
      qOut.reset(qRef);
      changeGain = false;
    }
    void stopMotion();
    void releaseMotion(double recover_time);
    void startTorque(double retrieve_time);
  };
  cnoid::BodyPtr robot;
  std::vector<Joint> joints;
  double recover_time = 2.5;
  double retrieve_time = 0.5;
  double servoon_time = 0.01;
  double servooff_time = 0.01;
};


extern "C"
{
  void EmergencyStopper2Init(RTC::Manager* manager);
};

#endif
