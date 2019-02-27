// -*- C++ -*-
/*!
 * @file  ArmImageGenerator.cpp
 * @brief Arm Image Generator RT Component(+depth)
 * @date $Date$
 *
 * $Id$
 */

#include "ArmImageGenerator.h"

#include <iomanip>
#include <iostream>
#include <fstream>
#include <ctime>


#define _USE_MATH_DEFINES
#include <math.h>

#ifdef WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#define RADIANS(x) (x)/180.0*M_PI


// #define NO_ARM_CONNECTION

// Module specification
// <rtc-template block="module_spec">
static const char* armimagegenerator_spec[] =
  {
    "implementation_id", "ArmImageGenerator",
    "type_name", "ArmImageGenerator",
    "description", "Arm Image Generator RT Component(for061)",
    "version", "1.0.0",
    "vendor", "kanamura",
    "category", "Experimental",
    "activity_type", "PERIODIC",
    "kind", "DataFlowComponent",
    "max_instance", "1",
    "language", "C++",
    "lang_type", "compile",
    // Configuration variables
    "conf.default.debug", "1",
    "conf.default.j0max", "1.57076",
    "conf.default.j1max", "1.57076",
    "conf.default.j0min", "-1.57076",
    "conf.default.j1min", "-1.57076",
    "conf.default.j0step", "0.157076",
    "conf.default.j1step", "0.157076",
    "conf.default.wait_interval", "1.5",
    "conf.default.camera_wait_time", "3.0",
    "conf.default.gripper_close_ratio", "0.6",

    "conf.default.z_min", "-0.072",
    "conf.default.z_prepare_offset", "0.030",
    "conf.default.y_prepare_offset", "0.00",
    "conf.default.x_prepare_offset", "0.00",
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.j0max", "text",
    "conf.__widget__.j1max", "text",
    "conf.__widget__.j0min", "text",
    "conf.__widget__.j1min", "text",
    "conf.__widget__.j0step", "text",
    "conf.__widget__.j1step", "text",

    "conf.__widget__.gripper_close_ratio", "slider.0.1",
    // Constraints
    "conf.__constraints__.gripper_close_ratio", "0.0<=x<=1.0",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
ArmImageGenerator::ArmImageGenerator(RTC::Manager* manager)
  // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    //m_rgbdCameraImageIn("rgbdCameraImage", m_rgbdCameraImage),
	m_rgbCameraImageIn0("rgbCameraImage0", m_rgbCameraImage0),
	m_rgbCameraImageIn1("rgbCameraImage1", m_rgbCameraImage1),
    m_manipCommonPort("manipCommon"),
    m_manipMiddlePort("manipMiddle")

    // </rtc-template>
  , m_jointPos(new JARA_ARM::JointPos())
{
}

/*!
 * @brief destructor
 */
ArmImageGenerator::~ArmImageGenerator()
{
}



RTC::ReturnCode_t ArmImageGenerator::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  //addInPort("rgbdCameraImage", m_rgbdCameraImageIn);
  addInPort("rgbCameraImage0", m_rgbCameraImageIn0);
  addInPort("rgbCameraImage1", m_rgbCameraImageIn1);

  // Set OutPort buffer

  // Set service provider to Ports

  // Set service consumers to Ports
  m_manipCommonPort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", m_manipCommon);
  m_manipMiddlePort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", m_manipMiddle);

  // Set CORBA Service Ports
  addPort(m_manipCommonPort);
  addPort(m_manipMiddlePort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "1");
  bindParameter("j0max", m_j0max, "1.57076");
  bindParameter("j1max", m_j1max, "1.57076");
  bindParameter("j0min", m_j0min, "-1.57076");
  bindParameter("j1min", m_j1min, "-1.57076");
  bindParameter("j0step", m_j0step, "0.157076");
  bindParameter("j1step", m_j1step, "0.157076");
  bindParameter("wait_interval", m_wait_interval, "1.5");
  bindParameter("camera_wait_time", m_camera_wait_time, "3.0");
  bindParameter("gripper_close_ratio", m_gripper_close_ratio, "0.6");

  bindParameter("z_min", m_z_min, "-0.072");
  bindParameter("x_prepare_offset", m_x_prepare_offset, "0.00");
  bindParameter("y_prepare_offset", m_y_prepare_offset, "0.00");
  bindParameter("z_prepare_offset", m_z_prepare_offset, "0.030");
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t ArmImageGenerator::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ArmImageGenerator::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ArmImageGenerator::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/


RTC::ReturnCode_t ArmImageGenerator::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "[ArmImageGenerator] Initializing Arm and Parameters" << std::endl;
  coil::TimeValue tv1(5.0);
  coil::sleep(tv1);

  std::cout << "[ArmImageGenerator] Waiting Arm Component is Activated....." << std::endl;
#ifndef NO_ARM_CONNECTION
  while (true) {
    //int ok_count = 0;
    const RTC::PortProfile& pp = m_manipCommonPort.getPortProfile();
    if (pp.connector_profiles.length() > 0) {
      RTC::PortProfile_var pp0 = pp.connector_profiles[0].ports[0]->get_port_profile();
      RTC::ComponentProfile_var cp0 = pp0->owner->get_component_profile();
      if (std::string(cp0->type_name) == armimagegenerator_spec[1]) {
	RTC::PortProfile_var pp1 = pp.connector_profiles[0].ports[1]->get_port_profile();
	RTC::ComponentProfile_var cp1 = pp1->owner->get_component_profile();
	RTC::ExecutionContext_var ec1 = pp1->owner->get_context(0);
	if (ec1->get_component_state(pp1->owner) == ACTIVE_STATE) {
	  break;
	}
      }
      else {
	RTC::ExecutionContext_var ec0 = pp0->owner->get_context(0);
	if (ec0->get_component_state(pp0->owner) == ACTIVE_STATE) {
	  break;
	}
      }
    }
  }



  JARA_ARM::RETURN_ID_var ret = m_manipCommon->servoON();
  if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
  }
  ret = m_manipMiddle->setSpeedJoint(30);
  if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
  }
#endif
  m_jointPos->length(6);
  m_jointPos[0] = 0;
  m_jointPos[1] = M_PI/4;
  m_jointPos[2] = M_PI/4;
  m_jointPos[3] = 0;
  m_jointPos[4] = M_PI/2;
  m_jointPos[5] = 0;

  //m_manipMiddle->movePTPJointAbs(m_jointPos);


  coil::TimeValue tv(3.0);
  coil::sleep(tv);


  m_j0counter = m_j1counter = 0;

  m_sleepTime = coil::TimeValue(m_wait_interval);
  std::cout << "[ArmImageGenerator] Wait " << m_sleepTime.sec() << "[sec], " << m_sleepTime.usec() << "[usec]" << std::endl;

  std::cout << "[ArmImageGenerator] Ready." << std::endl;

  time_t now = std::time(NULL);
  struct tm* localNow = std::localtime(&now);
  std::ostringstream ss;
  ss << "logs/log"
     << 1900 + localNow->tm_year
     << std::setw(2) << std::setfill('0') << localNow->tm_mon + 1
     << std::setw(2) << std::setfill('0') << localNow->tm_mday
     << std::setw(2) << std::setfill('0') << localNow->tm_hour
     << std::setw(2) << std::setfill('0') << localNow->tm_min
     << std::setw(2) << std::setfill('0') << localNow->tm_sec;

  m_logDir = ss.str();
#ifdef WIN32
  _mkdir(m_logDir.c_str());
#else
  mkdir(m_logDir.c_str(), 0777);
#endif


  std::ofstream configFile;
  configFile.open(m_logDir + "/config.yaml", std::ofstream::out);
  configFile << "j0min: " << m_j0min << std::endl;
  configFile << "j1min: " << m_j1min << std::endl;
  configFile << "j0max: " << m_j0max << std::endl;
  configFile << "j1max: " << m_j1max << std::endl;
  configFile << "j0step: " << m_j0step << std::endl;
  configFile << "j1step: " << m_j1step << std::endl;
  configFile << "wait_interval: " << m_wait_interval << std::endl;
  configFile.close();


  std::string filename = m_logDir + "/joints.csv";
  m_JointLog.open(filename.c_str(), std::ios::out);//, std::ofstream::out);

  std::string name = m_logDir + "/depth.csv";
  m_DepthLog.open(name.c_str(), std::ios::out);

  m_JointLog << "x, y, theta, ImageFilename, DepthImgFilename" << std::endl;

  m_DepthLog << "DepthData(480*360)" << std::endl;

  std::string b_filename = m_logDir + "/behavior.csv";
  m_BehaviorLog.open(b_filename.c_str(), std::ios::out);//, std::ofstream::out);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t ArmImageGenerator::onDeactivated(RTC::UniqueId ec_id)
{
  m_jointPos[0] = 0;
  m_jointPos[1] = 0;
  m_jointPos[2] = M_PI / 2;
  m_jointPos[4] = M_PI / 2;

#ifndef NO_ARM_CONNECTION
  JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPJointAbs(m_jointPos);
  if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
  }
  m_manipCommon->servoOFF();
#endif
  m_JointLog.close();
  m_BehaviorLog.close();

  coil::TimeValue tv(3.0);
  coil::sleep(tv);


  return RTC::RTC_OK;
}

double Uniform(void){
  return ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
}


bool ArmImageGenerator::moveOrigin(void) {
  m_BehaviorLog << "moveOrigin" << std::endl;
  JARA_ARM::CarPosWithElbow targetPos;
  targetPos.elbow = 0;
  targetPos.carPos[0][0] = -1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] =  0; targetPos.carPos[0][3] = 0.40;
  targetPos.carPos[1][0] =  0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] =  0; targetPos.carPos[1][3] =    0;
  targetPos.carPos[2][0] =  0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = -1; targetPos.carPos[2][3] = 0.30;
  JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPCartesianAbs(targetPos);
  if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    return true;
  }
  return false;
}

bool ArmImageGenerator::rotateX(double theta) {
  m_BehaviorLog << "rotateX(" << theta << ")" << std::endl;
  double c = cos(theta);
  double s = sin(theta);
  JARA_ARM::CarPosWithElbow targetPos;
  targetPos.elbow = 0;
  targetPos.carPos[0][0] = 1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = 0;
  targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = c; targetPos.carPos[1][2] = -s; targetPos.carPos[1][3] = 0;
  targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = s; targetPos.carPos[2][2] = c; targetPos.carPos[2][3] = 0;
  JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPCartesianRel(targetPos);
  if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    return true;
  }
  return false;
}

bool ArmImageGenerator::rotateY(double theta) {
  m_BehaviorLog << "rotateY(" << theta << ")" << std::endl;
  double c = cos(theta);
  double s = sin(theta);
  JARA_ARM::CarPosWithElbow targetPos;
  targetPos.elbow = 0;
  targetPos.carPos[0][0] = c; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = -s; targetPos.carPos[0][3] = 0;
  targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = 0;
  targetPos.carPos[2][0] = s; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = c; targetPos.carPos[2][3] = 0;
  JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPCartesianRel(targetPos);
  if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    return true;
  }
  return false;
}

bool ArmImageGenerator::rotateZ(double theta) {
  m_BehaviorLog << "rotateZ(" << theta << ")" << std::endl;
  double c = cos(theta);
  double s = sin(theta);
  JARA_ARM::CarPosWithElbow targetPos;
  targetPos.elbow = 0;
  targetPos.carPos[0][0] = c; targetPos.carPos[0][1] = -s; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = 0;
  targetPos.carPos[1][0] = s; targetPos.carPos[1][1] = c; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = 0;
  targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = 1; targetPos.carPos[2][3] = 0;
  JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPCartesianRel(targetPos);
  if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    return true;
  }
  return false;
}

bool ArmImageGenerator::moveTranslate(double dx, double dy, double dz) {
  m_BehaviorLog << "moveTranslate(" << dx << ", " << dy << ", " << dz << ")" << std::endl;
  JARA_ARM::CarPosWithElbow targetPos;
  targetPos.elbow = 0;
  //targetPos.carPos[0][0] = 1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = dx;
  //targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = dy;
  //targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = 1; targetPos.carPos[2][3] = dz;
  targetPos.carPos[0][0] = -1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] =  0; targetPos.carPos[0][3] = dx;
  targetPos.carPos[1][0] =  0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] =  0; targetPos.carPos[1][3] = dy;
  targetPos.carPos[2][0] =  0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = -1; targetPos.carPos[2][3] = dz;
  //JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPCartesianRel(targetPos);
  JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPCartesianAbs(targetPos);
  if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    return true;
  }
  return false;
}

JARA_ARM::CarPosWithElbow matrixProduct(JARA_ARM::CarPosWithElbow* T1, JARA_ARM::CarPosWithElbow* T2){
  double T1_m[4][4];
  double T2_m[4][4];
  double ans_m[4][4];

  printf("T1     : %4.4f %4.4f %4.4f %4.4f\n", T1->carPos[0][0], T1->carPos[0][1], T1->carPos[0][2], T1->carPos[0][3]);
  printf("         %4.4f %4.4f %4.4f %4.4f\n", T1->carPos[1][0], T1->carPos[1][1], T1->carPos[1][2], T1->carPos[1][3]);
  printf("         %4.4f %4.4f %4.4f %4.4f\n", T1->carPos[2][0], T1->carPos[2][1], T1->carPos[2][2], T1->carPos[2][3]);

  for (int i=0; i<4; i++){
    if (i < 3) {
      for (int j=0; j<4; j++){
        T1_m[i][j] = T1->carPos[i][j];
        T2_m[i][j] = T2->carPos[i][j];
      }
    } else {
      for (int j=0; j<4; j++){
        T1_m[i][j] = 0;
        T2_m[i][j] = 0;
      }
    }
  }

  for (int i=0; i<4; i++){
    for (int j=0; j<4; j++){
      double sum = 0;
      for (int k=0; k<4; k++){
        sum += T1_m[i][k] * T1_m[k][j];
      }
      ans_m[i][j] = sum;
    }
  }

  JARA_ARM::CarPosWithElbow product;
  for (int i=0; i<3; i++){
    for (int j=0; j<4; j++){
      product.carPos[i][j] = ans_m[i][j];
    }
  }

  printf("product: %4.4f %4.4f %4.4f %4.4f\n", product.carPos[0][0], product.carPos[0][1], product.carPos[0][2], product.carPos[0][3]);
  printf("         %4.4f %4.4f %4.4f %4.4f\n", product.carPos[1][0], product.carPos[1][1], product.carPos[1][2], product.carPos[1][3]);
  printf("         %4.4f %4.4f %4.4f %4.4f\n", product.carPos[2][0], product.carPos[2][1], product.carPos[2][2], product.carPos[2][3]);
  return product;
}


bool ArmImageGenerator::moveAbsWithPose3D(const RTC::Pose3D& poses) {
  std::cout << "move abs with pose 3d" << std::endl;
  m_BehaviorLog
    << "moveAbsWithPose3D("
    << "x=" << poses.position.x << ", "
    << "y=" << poses.position.y << ", "
    << "z=" << poses.position.z << ", "
    << "roll=" << poses.orientation.r << ", "
    << "yaw=" << poses.orientation.y << ", "
    << "pitch=" << poses.orientation.p << ")" << std::endl;

  // ここでロールピッチヨー表現の姿勢を変換行列にして
  // moveCartesianAbsに送る．

  JARA_ARM::CarPosWithElbow targetPos;
  JARA_ARM::CarPosWithElbow rotater;
  JARA_ARM::CarPosWithElbow tmpPos;
  targetPos.elbow = 0;
  rotater.elbow = 0;
  tmpPos.elbow = 0;

  // if any of poses.position has value over 1
  if (poses.position.x > 1 || poses.position.y > 1 || poses.position.z > 1) {
    std::cout << "Poses position are over 1. Re check the values"
              << "x = " << poses.position.x << ", "
              << "y = " << poses.position.y << ", "
              << "z = " << poses.position.z << std::endl;
    return false;
  }


  // Set XYZ coordinates and initial values of rotation matrix
  double x = poses.position.x;
  double y = poses.position.y;
  double z = poses.position.z;
  targetPos.carPos[0][0] = -1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] =  0; targetPos.carPos[0][3] = x;
  targetPos.carPos[1][0] =  0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] =  0; targetPos.carPos[1][3] = y;
  targetPos.carPos[2][0] =  0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = -1; targetPos.carPos[2][3] = z;
  tmpPos.carPos[0][3] = x;
  tmpPos.carPos[1][3] = y;
  tmpPos.carPos[2][3] = z;


  // Set roll values to rotater
  double roll = poses.orientation.r;
  rotater.carPos[0][0] =  cos(roll); rotater.carPos[0][1] = sin(roll); rotater.carPos[0][2] = 0;
  rotater.carPos[1][0] = -sin(roll); rotater.carPos[1][1] = cos(roll); rotater.carPos[1][2] = 0;
  rotater.carPos[2][0] =          0; rotater.carPos[2][1] =         0; rotater.carPos[2][2] = 1;
  // Compute product of rotater and targetPos and temporarily store it to tmpPos
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      double sum = 0;
      for (int k=0; k<3; k++){
        sum += rotater.carPos[i][k] * targetPos.carPos[k][j];
      }
      tmpPos.carPos[i][j] = sum;
    }
  }
  // Restore computd values to targetPos
  for (int i=0; i<3; i++){
    for (int j=0; j<4; j++){
      targetPos.carPos[i][j] = tmpPos.carPos[i][j];
    }
  }

  // Set yaw values to rotater
  double yaw = poses.orientation.y;
  rotater.carPos[0][0] = cos(yaw); rotater.carPos[0][1] = 0; rotater.carPos[0][2] = -sin(yaw);
  rotater.carPos[1][0] =        0; rotater.carPos[1][1] = 1; rotater.carPos[1][2] =         0;
  rotater.carPos[2][0] = sin(yaw); rotater.carPos[2][1] = 0; rotater.carPos[2][2] =  cos(yaw);
  // Compute product of rotater and targetPos and temporarily store it to tmpPos
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      double sum = 0;
      for (int k=0; k<3; k++){
        sum += rotater.carPos[i][k] * targetPos.carPos[k][j];
      }
      tmpPos.carPos[i][j] = sum;
    }
  }
  // Restore computd values to targetPos
  for (int i=0; i<3; i++){
    for (int j=0; j<4; j++){
      targetPos.carPos[i][j] = tmpPos.carPos[i][j];
    }
  }

  // Set pitch values to rotater
  double pitch = poses.orientation.p;
  rotater.carPos[0][0] = 1; rotater.carPos[0][1] =           0; rotater.carPos[0][2] =          0;
  rotater.carPos[1][0] = 0; rotater.carPos[1][1] =  cos(pitch); rotater.carPos[1][2] = sin(pitch);
  rotater.carPos[2][0] = 0; rotater.carPos[2][1] = -sin(pitch); rotater.carPos[2][2] = cos(pitch);
  // Compute product of rotater and targetPos and temporarily store it to tmpPos
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      double sum = 0;
      for (int k=0; k<3; k++){
        sum += rotater.carPos[i][k] * targetPos.carPos[k][j];
      }
      tmpPos.carPos[i][j] = sum;
    }
  }
  // Restore computd values to targetPos
  for (int i=0; i<3; i++){
    for (int j=0; j<4; j++){
      targetPos.carPos[i][j] = tmpPos.carPos[i][j];
    }
  }


  // Print the new set values
  printf("x: %f cm, ", poses.position.x * 100);
  printf("y: %f cm, ", poses.position.y * 100);
  printf("z: %f cm\n", poses.position.z * 100);
  printf("yaw: %f°, ",    poses.orientation.y / M_PI * 180);
  printf("pitch: %f°, ",  poses.orientation.p / M_PI * 180);
  printf("roll: %f°\n\n", poses.orientation.r / M_PI * 180);

  JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPCartesianAbs(targetPos);
  if (ret->id != JARA_ARM::OK) {
	  std::cout << "ERROR in ServoON" << std::endl;
	  std::cout << " ERRORCODE    :" << ret->id << std::endl;
	  std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
	  return true;
  }

  return true;
}

std::vector<RTC::Pose3D> ArmImageGenerator::generatePoses1() {
  m_BehaviorLog << "generatePoses()" << std::endl;
  std::vector<RTC::Pose3D> poses;

  // TODO: ここで撮影位置姿勢を生成し，posesに格納して返す
  // poses.position.x
  // poses.position.y
  // poses.position.z
  // poses.orientation.r ロール
  // poses.orientation.y ヨー
  // poses.orientation.p ピッチ

  double lenWristToCam = 11;  //座標位置からカメラ位置までの距離(cm)
	double x_offset = 30;
	double y_offset = 0;
	double z_offset = 17;
  double original_r = 8;
  double first_pitch = 230;

  // 1st Layer : phi == 50, divide 90 degrees into 2
  for(int th=-45; th<=45; th+=45){
    // Clock wise (view from top)
    double phi     = 50;
    double z_shift = original_r * sin(RADIANS(phi));
    double r       = original_r * cos(RADIANS(phi));

    double rol   = RADIANS(th);
    double rol_s = RADIANS(th+90); // roll shifted 90 degrees for ease in calculating new position
    double pit   = atan(tan(RADIANS(phi)) * sin(RADIANS(th)));
    double yaw   = atan(tan(RADIANS(phi)) * cos(RADIANS(th)));
    //   attr name  | conv |   origin  | translation on orbit | translate coordinates from wrist to hand
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (x_offset + r * sin(rol_s)       - lenWristToCam * sin(RADIANS(phi)) * cos(rol));
    pose.position.y = 0.01 * (y_offset + r * cos(rol_s)       + lenWristToCam * sin(RADIANS(phi)) * sin(rol));
    pose.position.z = 0.01 * (z_offset + z_shift              + lenWristToCam * cos(RADIANS(phi)));
    pose.orientation.r = rol;
    pose.orientation.p = pit;
    pose.orientation.y = yaw;
    poses.push_back(pose);
  }

  // 2nd Layer : phi == 40, divide 90 degrees into 3
  for(int th=60; th>=-60; th-=30){
    // Anti-clock wise (view from top)
    double phi     = 40;
    double z_shift = original_r * sin(RADIANS(phi));
    double r       = original_r * cos(RADIANS(phi));

    double rol   = RADIANS(th);
    double rol_s = RADIANS(th+90); // roll shifted 90 degrees for ease in calculating new position
    double pit   = atan(tan(RADIANS(phi)) * sin(RADIANS(th)));
    double yaw   = atan(tan(RADIANS(phi)) * cos(RADIANS(th)));
    //   attr name  | conv |   origin  | translation on orbit | translate coordinates from wrist to hand
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (x_offset + r * sin(rol_s)       - lenWristToCam * sin(RADIANS(phi)) * cos(rol));
    pose.position.y = 0.01 * (y_offset + r * cos(rol_s)       + lenWristToCam * sin(RADIANS(phi)) * sin(rol));
    pose.position.z = 0.01 * (z_offset + z_shift              + lenWristToCam * cos(RADIANS(phi)));
    pose.orientation.r = rol;
    pose.orientation.p = pit;
    pose.orientation.y = yaw;
    poses.push_back(pose);
  }

  // 3nd Layer : phi == 30, divide 90 degrees into 4
  for(int th=-67.5; th<=67.5; th+=22.5){
    // Clock wise (view from top)
    double phi     = 30;
    double z_shift = original_r * sin(RADIANS(phi));
    double r       = original_r * cos(RADIANS(phi));

    double rol   = RADIANS(th);
    double rol_s = RADIANS(th+90); // roll shifted 90 degrees for ease in calculating new position
    double pit   = atan(tan(RADIANS(phi)) * sin(RADIANS(th)));
    double yaw   = atan(tan(RADIANS(phi)) * cos(RADIANS(th)));
    //   attr name  | conv |   origin  | translation on orbit | translate coordinates from wrist to hand
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (x_offset + r * sin(rol_s)       - lenWristToCam * sin(RADIANS(phi)) * cos(rol));
    pose.position.y = 0.01 * (y_offset + r * cos(rol_s)       + lenWristToCam * sin(RADIANS(phi)) * sin(rol));
    pose.position.z = 0.01 * (z_offset + z_shift              + lenWristToCam * cos(RADIANS(phi)));
    pose.orientation.r = rol;
    pose.orientation.p = pit;
    pose.orientation.y = yaw;
    poses.push_back(pose);
  }

  // 4nd Layer : phi == 20, divide 90 degrees into 5
  for(int th=72; th>=-72; th-=18){
    // Anti-clock wise (view from top)
    double phi     = 20;
    double z_shift = original_r * sin(RADIANS(phi));
    double r       = original_r * cos(RADIANS(phi));

    double rol   = RADIANS(th);
    double rol_s = RADIANS(th+90); // roll shifted 90 degrees for ease in calculating new position
    double pit   = atan(tan(RADIANS(phi)) * sin(RADIANS(th)));
    double yaw   = atan(tan(RADIANS(phi)) * cos(RADIANS(th)));
    //   attr name  | conv |   origin  | translation on orbit | translate coordinates from wrist to hand
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (x_offset + r * sin(rol_s)       - lenWristToCam * sin(RADIANS(phi)) * cos(rol));
    pose.position.y = 0.01 * (y_offset + r * cos(rol_s)       + lenWristToCam * sin(RADIANS(phi)) * sin(rol));
    pose.position.z = 0.01 * (z_offset + z_shift              + lenWristToCam * cos(RADIANS(phi)));
    pose.orientation.r = rol;
    pose.orientation.p = pit;
    pose.orientation.y = yaw;
    poses.push_back(pose);
  }

  m_BehaviorLog << "generatePoses() (first half) ended." << std::endl;
  return poses;
}

std::vector<RTC::Pose3D> ArmImageGenerator::generatePoses2() {
  m_BehaviorLog << "generatePoses()" << std::endl;
  std::vector<RTC::Pose3D> poses;

  // TODO: ここで撮影位置姿勢を生成し，posesに格納して返す
  // poses.position.x
  // poses.position.y
  // poses.position.z
  // poses.orientation.r ロール
  // poses.orientation.y ヨー
  // poses.orientation.p ピッチ

  double lenWristToCam = 11;  //座標位置からカメラ位置までの距離(cm)
	double x_offset = 30;
	double y_offset = 0;
	double z_offset = 17;
  double original_r = 8;
  double first_pitch = 230;

  // 1st Layer : phi == 50, divide 90 degrees into 2
  for(int th=90; th>=-90; th-=45){
    // Anti-clock wise (view from top)
    double phi     = 50;
    double z_shift = original_r * sin(RADIANS(phi));
    double r       = original_r * cos(RADIANS(phi));

    double rol   = RADIANS(th);
    double rol_s = RADIANS(th+90); // roll shifted 90 degrees for ease in calculating new position
    double pit   = atan(tan(RADIANS(phi)) * sin(RADIANS(th)));
    double yaw   = atan(tan(RADIANS(phi)) * cos(RADIANS(th)));
    //   attr name  | conv |   origin  | translation on orbit | translate coordinates from wrist to hand
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (x_offset - r * sin(rol_s)       + lenWristToCam * sin(RADIANS(phi)) * cos(rol));
    pose.position.y = 0.01 * (y_offset - r * cos(rol_s)       - lenWristToCam * sin(RADIANS(phi)) * sin(rol));
    pose.position.z = 0.01 * (z_offset + z_shift              + lenWristToCam * cos(RADIANS(phi)));
    pose.orientation.r = rol;
    pose.orientation.p = -pit;
    pose.orientation.y = -yaw;
    poses.push_back(pose);
  }

  // 2nd Layer : phi == 40, divide 90 degrees into 3
  for(int th=-90; th<=90; th+=30){
    // Clock wise (view from top)
    double phi     = 40;
    double z_shift = original_r * sin(RADIANS(phi));
    double r       = original_r * cos(RADIANS(phi));

    double rol   = RADIANS(th);
    double rol_s = RADIANS(th+90); // roll shifted 90 degrees for ease in calculating new position
    double pit   = atan(tan(RADIANS(phi)) * sin(RADIANS(th)));
    double yaw   = atan(tan(RADIANS(phi)) * cos(RADIANS(th)));
    //   attr name  | conv |   origin  | translation on orbit | translate coordinates from wrist to hand
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (x_offset - r * sin(rol_s)       + lenWristToCam * sin(RADIANS(phi)) * cos(rol));
    pose.position.y = 0.01 * (y_offset - r * cos(rol_s)       - lenWristToCam * sin(RADIANS(phi)) * sin(rol));
    pose.position.z = 0.01 * (z_offset + z_shift              + lenWristToCam * cos(RADIANS(phi)));
    pose.orientation.r = rol;
    pose.orientation.p = -pit;
    pose.orientation.y = -yaw;
    poses.push_back(pose);
  }

  // 3nd Layer : phi == 30, divide 90 degrees into 4
  for(int th=90; th>=-90; th-=22.5){
    // Anti-clock wise (view from top)
    double phi     = 30;
    double z_shift = original_r * sin(RADIANS(phi));
    double r       = original_r * cos(RADIANS(phi));

    double rol   = RADIANS(th);
    double rol_s = RADIANS(th+90); // roll shifted 90 degrees for ease in calculating new position
    double pit   = atan(tan(RADIANS(phi)) * sin(RADIANS(th)));
    double yaw   = atan(tan(RADIANS(phi)) * cos(RADIANS(th)));
    //   attr name  | conv |   origin  | translation on orbit | translate coordinates from wrist to hand
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (x_offset - r * sin(rol_s)       + lenWristToCam * sin(RADIANS(phi)) * cos(rol));
    pose.position.y = 0.01 * (y_offset - r * cos(rol_s)       - lenWristToCam * sin(RADIANS(phi)) * sin(rol));
    pose.position.z = 0.01 * (z_offset + z_shift              + lenWristToCam * cos(RADIANS(phi)));
    pose.orientation.r = rol;
    pose.orientation.p = -pit;
    pose.orientation.y = -yaw;
    poses.push_back(pose);
  }

  // 4nd Layer : phi == 20, divide 90 degrees into 5
  for(int th=-90; th<=90; th+=18){
    // Clock wise (view from top)
    double phi     = 20;
    double z_shift = original_r * sin(RADIANS(phi));
    double r       = original_r * cos(RADIANS(phi));

    double rol   = RADIANS(th);
    double rol_s = RADIANS(th+90); // roll shifted 90 degrees for ease in calculating new position
    double pit   = atan(tan(RADIANS(phi)) * sin(RADIANS(th)));
    double yaw   = atan(tan(RADIANS(phi)) * cos(RADIANS(th)));
    //   attr name  | conv |   origin  | translation on orbit | translate coordinates from wrist to hand
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (x_offset - r * sin(rol_s)       + lenWristToCam * sin(RADIANS(phi)) * cos(rol));
    pose.position.y = 0.01 * (y_offset - r * cos(rol_s)       - lenWristToCam * sin(RADIANS(phi)) * sin(rol));
    pose.position.z = 0.01 * (z_offset + z_shift              + lenWristToCam * cos(RADIANS(phi)));
    pose.orientation.r = rol;
    pose.orientation.p = -pit;
    pose.orientation.y = -yaw;
    poses.push_back(pose);
  }

  m_BehaviorLog << "generatePoses() (second half) ended." << std::endl;
  return poses;
}

RTC::ReturnCode_t ArmImageGenerator::getJointAbs(std::vector<double>& joints) {
  joints.clear();

  JARA_ARM::JointPos_var pos(new JARA_ARM::JointPos);
  JARA_ARM::RETURN_ID_var retval;
  retval = m_manipCommon->getFeedbackPosJoint(pos);

  if (retval->id != JARA_ARM::OK) {
    std::cout << "ERROR: ArmImageGenerator::getJointAbs():" << retval->comment << std::endl;
    return RTC::RTC_ERROR;
  }
  if (pos->length() != 6) {
    std::cout << "ERROR: ArmImageGenerator::getJointAbs(): passed pos value does not have 6 elements but " << pos->length() << std::endl;
  }
  for(int i = 0;i < 6;i++) {
    joints.push_back(pos[i]);
  }
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ArmImageGenerator::moveJointAbs(const std::vector<double> joints) {
  if (joints.size() != 6) {
    std::cout << "ERROR: ArmImageGenerator::moveJointAbs(): Size of argument 'joints' must be 6 but " << joints.size() << std::endl;
    return RTC::RTC_ERROR;
  }

  JARA_ARM::JointPos pos;
  pos.length(6);
  for(int i = 0;i < 6;i++) {
    pos[i] = joints[i];
  }
  JARA_ARM::RETURN_ID_var retval;
  retval = m_manipMiddle->movePTPJointAbs(pos);
  if (retval->id != JARA_ARM::OK) {
    std::cout << "ERROR: ArmImageGenerator::moveJOintAbs(): " << retval->comment << std::endl;
    return RTC::RTC_ERROR;
  }
  return RTC::RTC_OK;
}

bool checkDirCreatable(std::string dirName){
  if (_mkdir (dirName.c_str()) == 0){
    return true;
  } else {
    return false;
  }
}

RTC::ReturnCode_t ArmImageGenerator::onMoveAutomatic() {
  coil::TimeValue tv(0.5);
  m_BehaviorLog << "onMoveAutomatic()" << std::endl;

  // ここで目標位置姿勢をリストにして受け取る
  std::cout << "Generate poses" << std::endl;
  std::vector<RTC::Pose3D> poseArray1 = generatePoses1();
  std::vector<RTC::Pose3D> poseArray2 = generatePoses2();

  //保存先のディレクトリのパスを設定する
  std::string dirName = "";
  for(int i=0; i<1000; i++){
    dirName = m_logDir + "/scene_" + std::to_string(i);
    if (checkDirCreatable(dirName))
      break;
  }

  //保存先にサブディレクトリ生成
  _mkdir((dirName + "/images").c_str());
  _mkdir((dirName + "/viewpoints").c_str());


  // ここで繰り返し移動して撮影する
  int count = 0;
  // 前半を撮影する
  std::cout << "Move : First half" << std::endl;
  for (auto pose : poseArray1) {
    // 移動
    moveAbsWithPose3D(pose);

    coil::sleep(tv);
    //画像保存
    saveImage(dirName + "/images/image_" + std::to_string(count) + ".png");
    //位置保存
    saveViewpoint(pose, dirName + "/viewpoints/viewpoint_" + std::to_string(count) + ".csv");

    saveLog(count++, pose);
    coil::sleep(tv);
  }

  //ユーザー対話
  std::cout << "Switch camera position" << std::ends;
  std::cout << "Enter any key when ready" << std::ends;
  char c;
  std::cin >> c;


  // 後半を撮影する
  for (auto pose : poseArray2) {
    // 移動
    moveAbsWithPose3D(pose);

    coil::sleep(tv);
    //画像保存
    saveImage(dirName + "/images/image_" + std::to_string(count) + ".png");
    //位置保存
    saveViewpoint(pose, dirName + "/viewpoints/viewpoint_" + std::to_string(count) + ".csv");

    saveLog(count++, pose);
    coil::sleep(tv);
  }

  m_BehaviorLog << "onMoveAutomatic() ended." << std::endl;
  return RTC::RTC_OK;
}

void ArmImageGenerator::saveImage(std::string savePath){
  /// Capture Image and Save
  bool imageArrived = false;

  //Inport data check
  while (!imageArrived){
	  while (m_rgbCameraImageIn0.isNew()){
		  m_rgbCameraImageIn0.read();
		  imageArrived = true;
	  }
  }
  std::cout << "[ArmImageGenerator] Image Arrived." << std::endl;

  long width = m_rgbCameraImage0.data.image.width;
  long height = m_rgbCameraImage0.data.image.height;
  long channels = (m_rgbCameraImage0.data.image.format == Img::CF_GRAY) ? 1 :
	  (m_rgbCameraImage0.data.image.format == Img::CF_RGB || m_rgbCameraImage0.data.image.format == Img::CF_PNG || m_rgbCameraImage0.data.image.format == Img::CF_JPEG) ? 3 :
	  (m_rgbCameraImage0.data.image.raw_data.length() / width / height);

  if (channels == 3)
	  m_buffer.create(height, width, CV_8UC3);
  else
	  m_buffer.create(height, width, CV_8UC1);

  long data_length = m_rgbCameraImage0.data.image.raw_data.length();

  if (m_rgbCameraImage0.data.image.format == Img::CF_RGB || m_rgbCameraImage0.data.image.format == Img::CF_GRAY) {
	  for (int i = 0; i < height; ++i)
		  memcpy(&m_buffer.data[i*m_buffer.step], &m_rgbCameraImage0.data.image.raw_data[i*width*channels], sizeof(unsigned char)*width*channels);
	  if (channels == 3)
		  cv::cvtColor(m_buffer, m_buffer, CV_RGB2BGR);
  }
  else if (m_rgbCameraImage0.data.image.format == Img::CF_JPEG || m_rgbCameraImage0.data.image.format == Img::CF_PNG) {
	  std::vector<uchar> compressed_image = std::vector<uchar>(data_length);
	  memcpy(&compressed_image[0], &m_rgbCameraImage0.data.image.raw_data[0], sizeof(unsigned char) * data_length);

	  //Decode received compressed image
	  cv::Mat decoded_image;
	  if (channels == 3) {
		  decoded_image = cv::imdecode(cv::Mat(compressed_image), CV_LOAD_IMAGE_COLOR);
		  cv::cvtColor(decoded_image, m_buffer, CV_RGB2BGR);
	  }
	  else {
		  decoded_image = cv::imdecode(cv::Mat(compressed_image), CV_LOAD_IMAGE_GRAYSCALE);
		  m_buffer = decoded_image;
	  }
  }
  cv::imwrite(savePath, m_buffer);

  std::cout << "saved image to : " << savePath << std::endl;
}

void ArmImageGenerator::saveViewpoint(RTC::Pose3D pose, std::string savePath){
	double x_offset = 27;
	double y_offset = 0;
	double z_offset = 22;

  double xFromCenter = pose.position.x - x_offset;
  double yFromCenter = pose.position.y - y_offset;
  double zFromCenter = pose.position.z - z_offset;

  double envYaw = pose.orientation.r;
  double envPitch = pose.orientation.p;

  std::ofstream outputfile(savePath);
  outputfile << xFromCenter << ","
              << yFromCenter << ","
              << zFromCenter << ","
              << envYaw      << ","
              << envPitch;
  outputfile.close();
}

void ArmImageGenerator::saveLog(int count, const RTC::Pose3D& targetPose) {
  /// TODO: ここでデータを保存します．
  m_BehaviorLog << "saveLog(" << count << ")" << std::endl;
  std::ostringstream ioss;
  std::string ext = ".png"; // 拡張子
  ioss << m_logDir << "/images/" << "image" << std::setw(4) << std::setfill('0') << count << ext;
  std::string imageFilename = ioss.str();

  std::ostringstream poss;
  ext = ".csv";
  poss << m_logDir << "/poses/" << "pose" << std::setw(4) << std::setfill('0') << count << ext;
  std::string poseFilename = poss.str();

  /// 画像ファイルの保存．今はダミー 1/10
  std::ofstream imFile(imageFilename);
  imFile.close();

  /// ポーズファイル．今はターゲットポーズを保存するけど，現在地を保存したいところ
  std::ofstream poseFile(poseFilename);
  poseFile << targetPose.position.x << ","
	   << targetPose.position.y << ","
	   << targetPose.position.z << ","
	   << targetPose.orientation.r << ","
	   << targetPose.orientation.p << ","
	   << targetPose.orientation.y << std::endl;
  poseFile.close();

  m_BehaviorLog << "saveLog() ended." << std::endl;
}



RTC::ReturnCode_t ArmImageGenerator::onExecute(RTC::UniqueId ec_id)
{

  JARA_ARM::CarPosWithElbow targetPos;
  JARA_ARM::CarPosWithElbow movePos;
  JARA_ARM::CarPosWithElbow tmpPos;
  targetPos.elbow = 0;
  targetPos.carPos[0][0] = 1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = 0;
  targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = 0;
  targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = 1; targetPos.carPos[2][3] = 0;

  std::cout << "Input Command (h for help):" << std::ends;
  char c;
  std::cin >> c;
  int n = 0;
  if (c != 'a') { // 'a' コマンドは自動動作用なのでnの値はいらない
    std::cout << "Input Amount:" << std::ends;
    std::string tmp;
    std::cin >> tmp;
    n = atoi(tmp.c_str());
  }
  rewind(stdin);
  fflush(stdin);

  JARA_ARM::CarPosWithElbow_var pos = new JARA_ARM::CarPosWithElbow();//(new JARA_ARM::CarPosWithElbow_var());

  coil::TimeValue tv(1.0);
  JARA_ARM::RETURN_ID_var ret;
  std::vector<double> joints;

  bool getImage = false;

/*
  double pitch   = 90;
  double c_pitch = cos(RADIANS(pitch));
  double s_pitch = sin(RADIANS(pitch));
  double roll    = 0;
  double c_roll  = cos(RADIANS( roll));
  double s_roll  = sin(RADIANS( roll));
  double yaw     = 0;
  double c_yaw   = cos(RADIANS(  yaw));
  double s_yaw   = sin(RADIANS(  yaw));
  */


  double roll = -45;
  double phi = -40;
  double yaw = atan(tan(RADIANS(phi)) * cos(RADIANS(roll))) / M_PI * 180;
  double pit = atan(tan(RADIANS(phi)) * sin(RADIANS(roll))) / M_PI * 180;

  switch (c) {
  case 'a' : // a ならば自動動作をしてonExecuteを返す
    std::cout << "moveAutomatic" << std::endl;
    return onMoveAutomatic();

    std::cout << "Test matrix move" << std::endl;

    std::cout << "roll: " << roll << std::endl;
    std::cout << "yaw : " << yaw  << std::endl;
    std::cout << "pit : " << pit  << std::endl;


    m_manipMiddle->getFeedbackPosCartesian(pos);

    for (int i=0; i<3; i++){
      for (int j=0; j<4; j++){
        targetPos.carPos[i][j] = pos->carPos[i][j];
      }
    }
    //starting position
    targetPos.carPos[0][0] = -1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] =  0; targetPos.carPos[0][3] = 0.2;
    targetPos.carPos[1][0] =  0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] =  0; targetPos.carPos[1][3] = 0;
    targetPos.carPos[2][0] =  0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = -1; targetPos.carPos[2][3] = 0.3;

    //roll
    movePos.elbow = 0;
    movePos.carPos[0][0] =  cos(RADIANS(-40)); movePos.carPos[0][1] = sin(RADIANS(-40)); movePos.carPos[0][2] = 0; movePos.carPos[0][3] = 0;
    movePos.carPos[1][0] = -sin(RADIANS(-40)); movePos.carPos[1][1] = cos(RADIANS(-40)); movePos.carPos[1][2] = 0; movePos.carPos[1][3] = 0;
    movePos.carPos[2][0] =                  0; movePos.carPos[2][1] =                 0; movePos.carPos[2][2] = 1; movePos.carPos[2][3] = 0;

    tmpPos.elbow = 0;
    for (int i=0; i<3; i++){
      for (int j=0; j<3; j++){
        double sum = 0;
        for (int k=0; k<3; k++){
          sum += movePos.carPos[i][k] * targetPos.carPos[k][j];
        }
        tmpPos.carPos[i][j] = sum;
      }
    }

    for (int i=0; i<3; i++){
      for (int j=0; j<4; j++){
        targetPos.carPos[i][j] = tmpPos.carPos[i][j];
      }
    }

    //yaw
    movePos.carPos[0][0] = cos(RADIANS(-30)); movePos.carPos[0][1] = 0; movePos.carPos[0][2] = -sin(RADIANS(-30)); movePos.carPos[0][3] = 0;
    movePos.carPos[1][0] =                 0; movePos.carPos[1][1] = 1; movePos.carPos[1][2] =                  0; movePos.carPos[1][3] = 0;
    movePos.carPos[2][0] = sin(RADIANS(-30)); movePos.carPos[2][1] = 0; movePos.carPos[2][2] =  cos(RADIANS(-30)); movePos.carPos[2][3] = 0;

    tmpPos.elbow = 0;
    for (int i=0; i<3; i++){
      for (int j=0; j<3; j++){
        double sum = 0;
        for (int k=0; k<3; k++){
          sum += movePos.carPos[i][k] * targetPos.carPos[k][j];
        }
        tmpPos.carPos[i][j] = sum;
      }
    }

    for (int i=0; i<3; i++){
      for (int j=0; j<4; j++){
        targetPos.carPos[i][j] = tmpPos.carPos[i][j];
      }
    }

    //pitch
    movePos.carPos[0][0] = 1; movePos.carPos[0][1] =                 0; movePos.carPos[0][2] =                0; movePos.carPos[0][3] = 0;
    movePos.carPos[1][0] = 0; movePos.carPos[1][1] =  cos(RADIANS(30)); movePos.carPos[1][2] = sin(RADIANS(30)); movePos.carPos[1][3] = 0;
    movePos.carPos[2][0] = 0; movePos.carPos[2][1] = -sin(RADIANS(30)); movePos.carPos[2][2] = cos(RADIANS(30)); movePos.carPos[2][3] = 0;

    tmpPos.elbow = 0;
    for (int i=0; i<3; i++){
      for (int j=0; j<3; j++){
        double sum = 0;
        for (int k=0; k<3; k++){
          sum += movePos.carPos[i][k] * targetPos.carPos[k][j];
        }
        tmpPos.carPos[i][j] = sum;
      }
    }


    //tmpPos.carPos[0][3] = pos->carPos[0][3] - 0.2;
    //tmpPos.carPos[1][3] = pos->carPos[1][3];
    //tmpPos.carPos[2][3] = pos->carPos[2][3];
    tmpPos.carPos[0][3] = 0.2;
    tmpPos.carPos[1][3] = 0;
    tmpPos.carPos[2][3] = 0.3;

    printf("tmpPos %4.4f %4.4f %4.4f %4.4f\n", tmpPos.carPos[0][0], tmpPos.carPos[0][1], tmpPos.carPos[0][2], tmpPos.carPos[0][3]);
    printf("       %4.4f %4.4f %4.4f %4.4f\n", tmpPos.carPos[1][0], tmpPos.carPos[1][1], tmpPos.carPos[1][2], tmpPos.carPos[1][3]);
    printf("       %4.4f %4.4f %4.4f %4.4f\n", tmpPos.carPos[2][0], tmpPos.carPos[2][1], tmpPos.carPos[2][2], tmpPos.carPos[2][3]);

    ret = m_manipMiddle->movePTPCartesianAbs(tmpPos);
    if (ret->id != JARA_ARM::OK) {
      std::cout << "ERROR in ServoON" << std::endl;
      std::cout << " ERRORCODE    :" << ret->id << std::endl;
      std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    }

    break;

  case '0':
    std::cout << "reset" << std::endl;
    moveOrigin();
    break;

  case 'l':
    std::cout << "getFeedbackPosCartesian" << std::endl;
#ifndef NO_ARM_CONNECTION
    m_manipMiddle->getFeedbackPosCartesian(pos);
    printf("carPos: %4.4f %4.4f %4.4f %4.4f\n", pos->carPos[0][0], pos->carPos[0][1], pos->carPos[0][2], pos->carPos[0][3]);
    printf("        %4.4f %4.4f %4.4f %4.4f\n", pos->carPos[1][0], pos->carPos[1][1], pos->carPos[1][2], pos->carPos[1][3]);
    printf("        %4.4f %4.4f %4.4f %4.4f\n", pos->carPos[2][0], pos->carPos[2][1], pos->carPos[2][2], pos->carPos[2][3]);
#endif
    break;

  case 'o':
    std::cout << "servoON" << std::endl;
#ifndef NO_ARM_CONNECTION
    m_manipCommon->servoON();
#endif
    break;

  case 'i':
    std::cout << "servoOFF" << std::endl;
#ifndef NO_ARM_CONNECTION
    m_manipCommon->servoOFF();
#endif
    break;

  case 'f':
    std::cout << "move right" << std::endl;
    if (moveTranslate(0, -0.01*n, 0)) return RTC::RTC_ERROR;
    break;

  case 's':
    std::cout << "move left" << std::endl;
    if (moveTranslate(0, 0.01*n, 0)) return RTC::RTC_ERROR;
    break;

  case 'e':
    std::cout << "move forward" << std::endl;
    if (moveTranslate(0.01*n, 0, 0)) return RTC::RTC_ERROR;
    break;

  case 'c':
    std::cout << "move backward" << std::endl;
    if (moveTranslate(-0.01*n, 0, 0)) return RTC::RTC_ERROR;
    break;

  case 'w':
    std::cout << "Up" << std::endl;
    if (moveTranslate(0, 0, 0.01*n)) return RTC::RTC_ERROR;
    break;

  case 'r':
    std::cout << "Down" << std::endl;
    if (moveTranslate(0, 0, -0.01*n)) return RTC::RTC_ERROR;
    break;

  case 'x':
    std::cout << "rotateCW" << std::endl;
    if (rotateZ(RADIANS(n))) return RTC::RTC_ERROR;
    break;

  case 'v':
    std::cout << "rotateCCW" << std::endl;
    if (rotateZ(RADIANS(-n))) return RTC::RTC_ERROR;
    break;

  case 'y':
    std::cout << "Close Gripper" << std::endl;
    m_manipMiddle->closeGripper();
    break;

  case 'n':
    std::cout << "Open Gripper" << std::endl;
    m_manipMiddle->openGripper();
    break;

  case 'p':
    m_manipMiddle->setSpeedCartesian(10);
    break;

  case ';':
    m_manipMiddle->setSpeedCartesian(200);
    break;

  case 'j':
    std::cout << "rotateJ" << std::endl;
    if (rotateY(RADIANS(-n))) return RTC::RTC_ERROR;
    break;

  case 'k':
    std::cout << "inverseJ" << std::endl;
    if (rotateY(RADIANS(n))) return RTC::RTC_ERROR;
    break;

  case ',':
    std::cout << "rotate," << std::endl;
    if (rotateX(RADIANS(-n))) return RTC::RTC_ERROR;
    break;

  case '.':
    std::cout << "inverse," << std::endl;
    if (rotateX(RADIANS(n))) return RTC::RTC_ERROR;
    break;

  case 'z':
	  std::cout << "get joint_pos" << std::endl;
	  getJointAbs(joints);
	  for (auto x: joints){
		  std::cout << x << std::endl;
	  }
	  break;

  case '1':
	  std::cout << "move by joint_pos" << std::endl;
	  getJointAbs(joints);
	  joints[3] = RADIANS(n);
	  moveJointAbs(joints);
	  for (auto x: joints){
		  std::cout << x/M_PI*180 << std::endl;
	  }
    std::cout << "Keep last joint to 45 degrees" << std::endl;
	  joints[5] = RADIANS(90);
	  moveJointAbs(joints);
    std::cout << "Done" << std::endl;
	  break;

  case '2':
	  saveImage("./test_image.png");
	  break;

  default:
    printf("Unknown Command %c\n", c);
    break;
  }

  return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t ArmImageGenerator::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ArmImageGenerator::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ArmImageGenerator::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ArmImageGenerator::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ArmImageGenerator::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

extern "C"
{
  void ArmImageGeneratorInit(RTC::Manager* manager)
  {
    coil::Properties profile(armimagegenerator_spec);
    manager->registerFactory(profile,
			     RTC::Create<ArmImageGenerator>,
			     RTC::Delete<ArmImageGenerator>);
  }
};