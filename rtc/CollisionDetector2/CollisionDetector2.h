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

  bool setTolerance(const char *i_link_pair_name, double i_tolerance);
  bool setCollisionLoop(int input_loop);
  bool getCollisionStatus(OpenHRP::CollisionDetectorService::CollisionState &state);

  bool checkIsSafeTransition(void);
  bool enable(void);
  bool disable(void);

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
    CollisionLinkPair(cnoid::LinkPtr link0_, cnoid::LinkPtr link1_) : link0(link0_), link1(link1_), point0(cnoid::Vector3(0,0,0)), point1(cnoid::Vector3(0,0,0)), distance(0), tolerance(0) {
    }
    cnoid::LinkPtr link0, link1;
    cnoid::Vector3 point0, point1; // world coords
    double distance;
    double tolerance;
  };
  std::unordered_map<cnoid::LinkPtr, std::shared_ptr<Vclip::Polyhedron> > m_VclipLinks;
  cnoid::BodyPtr m_robot;
  std::map<std::string, std::shared_ptr<CollisionLinkPair> > m_pair;

  std::vector<bool> m_link_collision;
  unsigned int m_debugLevel;
  bool m_enable;

  OpenHRP::CollisionDetectorService::CollisionState m_state;

  bool collision_mode;
  cnoid::VectorXd m_stop_jointdata;
  bool m_safe_posture;
};

extern "C"
{
  void CollisionDetector2Init(RTC::Manager* manager);
};

#endif // COLLISION_DETECTOR2_H
