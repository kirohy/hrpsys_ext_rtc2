#ifndef COLLISION_DETECTOR2_H
#define COLLISION_DETECTOR2_H

#include <rtm/idl/BasicDataType.hh>
#include "hrpsys/idl/HRPDataTypes.hh"
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <cnoid/Body>
#include <choreonoid_vclip/choreonoid_vclip.h>
#include <unordered_map>
#include <mutex>

#include "CollisionDetector2Service_impl.h"

class CollisionDetector2
  : public RTC::DataFlowComponentBase
{
 public:
  CollisionDetector2(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool enable(void);
  bool disable(void);
  bool setCollisionDetector2Param(const hrpsys_ext_rtc::CollisionDetector2Service::CollisionDetector2Param& i_param);
  bool getCollisionDetector2Param(hrpsys_ext_rtc::CollisionDetector2Service::CollisionDetector2Param& i_param);

 protected:
  RTC::TimedDoubleSeq m_qRef;
  RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
  RTC::TimedDoubleSeq m_qCurrent;
  RTC::InPort<RTC::TimedDoubleSeq> m_qCurrentIn;
  OpenHRP::TimedLongSeqSeq m_servoState;
  RTC::InPort<OpenHRP::TimedLongSeqSeq> m_servoStateIn;

  RTC::TimedLong m_stopSignal;
  RTC::OutPort<RTC::TimedLong> m_stopSignalOut;
  RTC::TimedLong m_releaseSignal;
  RTC::OutPort<RTC::TimedLong> m_releaseSignalOut;

  RTC::CorbaPort m_CollisionDetector2ServicePort;

  CollisionDetector2Service_impl m_service0;

  void setupVClipModel(cnoid::BodyPtr i_body);
  void setupVClipModel(cnoid::Link *i_link);

 private:
  class CollisionLinkPair {
  public:
    CollisionLinkPair(cnoid::LinkPtr link0_, cnoid::LinkPtr link1_) : link0(link0_), link1(link1_), currentDistance(0), nextDistance(0) {
    }
    cnoid::LinkPtr link0, link1;
    double currentDistance;
    double nextDistance;
  };
  std::unordered_map<cnoid::LinkPtr, std::shared_ptr<Vclip::Polyhedron> > m_VclipLinks;
  cnoid::BodyPtr m_robot;
  std::map<std::string, std::shared_ptr<CollisionLinkPair> > m_pair;

  bool m_enable;
  long loop = 0;
  std::mutex mutex_;

  cnoid::VectorXd m_qRefv;
  cnoid::VectorXd m_qCurrentv;
  std::vector<bool> m_servoStatev;
  bool m_collisionFreeOnce;
  bool prevCollision;

  double tolerance = 0.005;
  double recover_time = 1.0;
};

extern "C"
{
  void CollisionDetector2Init(RTC::Manager* manager);
};

#endif // COLLISION_DETECTOR2_H
