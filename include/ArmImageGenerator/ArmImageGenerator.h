// -*- C++ -*-
/*!
 * @file  ArmImageGenerator.h
 * @brief Arm Image Generator RT Component(+depth)
 * @date  $Date$
 *
 * $Id$
 */

#ifndef ARMIMAGEGENERATOR_H
#define ARMIMAGEGENERATOR_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "ManipulatorCommonInterface_CommonStub.h"
#include "ManipulatorCommonInterface_MiddleLevelStub.h"
#include "DepthCameraStub.h"

// </rtc-template>


#include <opencv2/opencv.hpp>
// Service Consumer stub headers
// <rtc-template block="port_stub_h">
using namespace RGBDCamera;
// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

using namespace RTC;

/*!
 * @class ArmImageGenerator
 * @brief Arm Image Generator RT Component(+depth)
 *
 */
class ArmImageGenerator
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  ArmImageGenerator(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~ArmImageGenerator();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  debug
   * - DefaultValue: 1
   */
  int m_debug;
  /*!
   * 
   * - Name:  j0max
   * - DefaultValue: 1.57076
   */
  float m_j0max;
  /*!
   * 
   * - Name:  j1max
   * - DefaultValue: 1.57076
   */
  float m_j1max;
  /*!
   * 
   * - Name:  j0min
   * - DefaultValue: -1.57076
   */
  float m_j0min;
  /*!
   * 
   * - Name:  j1min
   * - DefaultValue: -1.57076
   */
  float m_j1min;
  /*!
   * 
   * - Name:  j0step
   * - DefaultValue: 0.157076
   */
  float m_j0step;
  /*!
   * 
   * - Name:  j1step
   * - DefaultValue: 0.157076
   */
  float m_j1step;
  /*!
   * 
   * - Name:  wait_interval
   * - DefaultValue: 1.0
   */
  float m_wait_interval;
  /*!
   * 
   * - Name:  camera_wait_time
   * - DefaultValue: 3.0
   */
  float m_camera_wait_time ;
  /*!
   * 
   * - Name:  gripper_close_ratio
   * - DefaultValue: 0.1
   */
  float m_gripper_close_ratio;


  float m_z_min;
  float m_z_prepare_offset;
  float m_y_prepare_offset;
  float m_x_prepare_offset;

  float m_camera_jointPos0;
  float m_camera_jointPos1;
  float m_camera_jointPos2;
  float m_camera_jointPos3;
  float m_camera_jointPos4;
  float m_camera_jointPos5;

  float m_initial_jointPos0;
  float m_initial_jointPos1;
  float m_initial_jointPos2;
  float m_initial_jointPos3;
  float m_initial_jointPos4;
  float m_initial_jointPos5;
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RGBDCamera::TimedRGBDCameraImage m_rgbdCameraImage;
  /*!
   */
  InPort<RGBDCamera::TimedRGBDCameraImage> m_rgbdCameraImageIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  /*!
   */
  RTC::CorbaPort m_manipCommonPort;
  /*!
   */
  RTC::CorbaPort m_manipMiddlePort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  /*!
   */
  RTC::CorbaConsumer<JARA_ARM::ManipulatorCommonInterface_Common> m_manipCommon;
  /*!
   */
  RTC::CorbaConsumer<JARA_ARM::ManipulatorCommonInterface_Middle> m_manipMiddle;
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>
  
  	 JARA_ARM::JointPos_var m_jointPos;
	 int m_j0counter;
	 int m_j1counter;

	 coil::TimeValue m_sleepTime;

	 cv::Mat m_buffer;

	 std::ofstream m_JointLog;

  	 std::ofstream m_DepthLog;

  std::ofstream m_BehaviorLog;
  
	 std::string m_logDir;

  // float  m_depth; not used


  // 手先をデカルト座標系で移動
  bool moveOrigin(); // 原点に移動
  bool moveTranslate(double dx, double dy, double dz); // 並進移動．姿勢変化はなし
  bool rotateX(double theta); // 回転移動．X軸回転．thetaは差分
  bool rotateY(double theta); // 回転移動．Y軸回転．thetaは差分
  bool rotateZ(double theta); // 回転移動．Z軸回転．thetaは差分


  RTC::ReturnCode_t onMoveAutomatic();
  bool moveAbsWithPose3D(const RTC::Pose3D& pose);
  std::vector<RTC::Pose3D> generatePoses();
  void saveLog(int count, const RTC::Pose3D& targetPose);

  /*
   * 関節角度取得
   *
   * @param joints 関節角度を格納するvector．中身はクリアされ，6つの関節のデータが格納される．
   * @return RTC_OKで成功．失敗時はjointsの内容は不定
   */
  RTC::ReturnCode_t getJointAbs(std::vector<double>& joints);
  
  /**
   * 関節角度制御
   *
   * @param joints 関節角度．jointsは6つの値．サイズが違う場合は失敗する
   * @return RTC_OKで成功
   */
  RTC::ReturnCode_t moveJointAbs(const std::vector<double> joints);

  /**
   * 座標の誤差修正（関節角度は無視）
   *
   * @param poseは到達したい絶対座標
   * @return trueで成功
   */
  bool fixPosError(const RTC::Pose3D pose);
};


extern "C"
{
  DLL_EXPORT void ArmImageGeneratorInit(RTC::Manager* manager);
};

#endif // ARMIMAGEGENERATOR_H
