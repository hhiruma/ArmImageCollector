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
//#include <fstream>
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
    m_rgbdCameraImageIn("rgbdCameraImage", m_rgbdCameraImage),
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
  addInPort("rgbdCameraImage", m_rgbdCameraImageIn);

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
  targetPos.carPos[0][0] = -1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = 0.40;
  targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = 0;
  targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = -1; targetPos.carPos[2][3] = 0.30;
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
  targetPos.carPos[0][0] = -1+c; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = -s; targetPos.carPos[0][3] = 0.2;
  targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = 0;
  targetPos.carPos[2][0] = s; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = -1+c; targetPos.carPos[2][3] = 0.3;
  JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPCartesianAbs(targetPos);
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
  targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = -1; targetPos.carPos[2][3] = 0;
  JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPCartesianAbs(targetPos);
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
  targetPos.carPos[0][0] = 1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = dx;
  targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = dy;
  targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = 1; targetPos.carPos[2][3] = dz;
  JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPCartesianRel(targetPos);
  if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    return true;
  }
  return false;
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
  targetPos.elbow = 0;
  //translate position
  targetPos.carPos[0][0] = -1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] =  0; targetPos.carPos[0][3] = poses.position.x;
  targetPos.carPos[1][0] =  0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] =  0; targetPos.carPos[1][3] = poses.position.y;
  targetPos.carPos[2][0] =  0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = -1; targetPos.carPos[2][3] = poses.position.z;
  

  //rotate yaw
  double c_yaw = cos(poses.orientation.y);
  double s_yaw = sin(poses.orientation.y);
  targetPos.carPos[0][0] += c_yaw; targetPos.carPos[0][1] += 0; targetPos.carPos[0][2] += -s_yaw;
  targetPos.carPos[1][0] +=     0; targetPos.carPos[1][1] += 0; targetPos.carPos[1][2] +=      0;
  targetPos.carPos[2][0] += s_yaw; targetPos.carPos[2][1] += 0; targetPos.carPos[2][2] +=  c_yaw;

  //rotate roll
  double c_roll = cos(poses.orientation.r);
  double s_roll = sin(poses.orientation.r);
  targetPos.carPos[0][0] += c_roll; targetPos.carPos[0][1] += -s_roll; targetPos.carPos[0][2] += 0;
  targetPos.carPos[1][0] += s_roll; targetPos.carPos[1][1] +=  c_roll; targetPos.carPos[1][2] += 0;
  targetPos.carPos[2][0] +=      0; targetPos.carPos[2][1] +=       0; targetPos.carPos[2][2] += 0;
  
  printf("tarPos: %4.4f %4.4f %4.4f %4.4f\n", targetPos.carPos[0][0], targetPos.carPos[0][1], targetPos.carPos[0][2], targetPos.carPos[0][3]);
  printf("        %4.4f %4.4f %4.4f %4.4f\n", targetPos.carPos[1][0], targetPos.carPos[1][1], targetPos.carPos[1][2], targetPos.carPos[1][3]);
  printf("        %4.4f %4.4f %4.4f %4.4f\n", targetPos.carPos[2][0], targetPos.carPos[2][1], targetPos.carPos[2][2], targetPos.carPos[2][3]);


  JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPCartesianAbs(targetPos);
  if (ret->id != JARA_ARM::OK) {
	  std::cout << "ERROR in ServoON" << std::endl;
	  std::cout << " ERRORCODE    :" << ret->id << std::endl;
	  std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
	  return true;
  }

  m_BehaviorLog << "moveAbsWithPose3D() ended." << std::endl;
  return true;
}

std::vector<RTC::Pose3D> ArmImageGenerator::generatePoses() {
  m_BehaviorLog << "generatePoses()" << std::endl;
  std::vector<RTC::Pose3D> poses;

  // TODO: ここで撮影位置姿勢を生成し，posesに格納して返す
  // poses.position.x
  // poses.position.y
  // poses.position.z
  // poses.orientation.r ロール
  // poses.orientation.y ヨー
  // poses.orientation.p ピッチ

  double distToCam = 5;  //座標位置からカメラ位置までの距離(cm)

  //first layer
  for(int roll = 0; roll<=360; roll+=90){
	int yaw = 245;
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (20 + 3.38 * sin(RADIANS(roll/2)) - distToCam * sin(RADIANS((yaw-180)/2)) * cos(RADIANS(roll/2)));
    pose.position.y = 0.01 * (0  + 3.38 * cos(RADIANS(roll/2)) - distToCam * sin(RADIANS((yaw-180)/2)) * sin(RADIANS(roll/2)));
    pose.position.z = 0.01 * (30                                     + distToCam * cos(RADIANS((yaw-180)/2)));
	pose.orientation.r = RADIANS(roll);
    pose.orientation.p = 0;
    pose.orientation.y = RADIANS(yaw);
    poses.push_back(pose);
  }
  for(int roll = 270; roll<=450; roll+=90){
	int yaw = 115;
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (20 - 3.38 * sin(RADIANS(roll/2)) + distToCam * sin(RADIANS((yaw-180)/2)) * cos(RADIANS(roll/2)));
    pose.position.y = 0.01 * (0  - 3.38 * cos(RADIANS(roll/2)) + distToCam * sin(RADIANS((yaw-180)/2)) * sin(RADIANS(roll/2)));
    pose.position.z = 0.01 * (30                                     + distToCam * cos(RADIANS((yaw-180)/2)));
	pose.orientation.r = RADIANS(roll);
    pose.orientation.p = 0;
	pose.orientation.y = RADIANS(yaw);
    poses.push_back(pose);
  }

  //second layer
  for(int roll = 180; roll<=540; roll+=60){
	int yaw = 220;
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (20 + 6.13 * sin(RADIANS(roll/2)) - distToCam * sin(RADIANS((yaw-180)/2)) * cos(RADIANS(roll/2)));
    pose.position.y = 0.01 * (0  + 6.13 * cos(RADIANS(roll/2)) - distToCam * sin(RADIANS((yaw-180)/2)) * sin(RADIANS(roll/2)));
    pose.position.z = 0.01 * (30 - 2.11                              + distToCam * cos(RADIANS((yaw-180)/2)));
	pose.orientation.r = RADIANS(roll);
    pose.orientation.p = 0;
	pose.orientation.y = RADIANS(yaw);
    poses.push_back(pose);
  }
  for(int roll = 240; roll<=480; roll+=60){
	int yaw = 140;
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (20 - 6.13 * sin(RADIANS(roll/2)) + distToCam * sin(RADIANS((yaw-180)/2)) * cos(RADIANS(roll/2)));
    pose.position.y = 0.01 * (0  - 6.13 * cos(RADIANS(roll/2)) + distToCam * sin(RADIANS((yaw-180)/2)) * sin(RADIANS(roll/2)));
    pose.position.z = 0.01 * (30 - 2.11                              + distToCam * cos(RADIANS((yaw-180)/2)));
	pose.orientation.r = RADIANS(roll);
    pose.orientation.p = 0;
	pose.orientation.y = RADIANS(yaw);
    poses.push_back(pose);
  }

  //third layer
  for(int roll = 180; roll<=540; roll+=36){
	int yaw = 195;
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (20 + 7.73 * sin(RADIANS(roll/2)) - distToCam * sin(RADIANS((yaw-180)/2)) * cos(RADIANS(roll/2)));
    pose.position.y = 0.01 * (0  + 7.73 * cos(RADIANS(roll/2)) - distToCam * sin(RADIANS((yaw-180)/2)) * sin(RADIANS(roll/2)));
    pose.position.z = 0.01 * (30 - 5.18                              + distToCam * cos(RADIANS((yaw-180)/2)));
	pose.orientation.r = RADIANS(roll);
    pose.orientation.p = 0;
	pose.orientation.y = RADIANS(yaw);
    poses.push_back(pose);
  }
  for(int roll = 216; roll<=504; roll+=36){
	int yaw = 165;
    RTC::Pose3D pose;
    pose.position.x = 0.01 * (20 - 7.73 * sin(RADIANS(roll/2)) + distToCam * sin(RADIANS((yaw-180)/2)) * cos(RADIANS(roll/2)));
    pose.position.y = 0.01 * (0  - 7.73 * cos(RADIANS(roll/2)) + distToCam * sin(RADIANS((yaw-180)/2)) * sin(RADIANS(roll/2)));
    pose.position.z = 0.01 * (30 - 5.18                              + distToCam * cos(RADIANS((yaw-180)/2)));
	pose.orientation.r = RADIANS(roll);
    pose.orientation.p = 0;
    pose.orientation.y = RADIANS(yaw);
    poses.push_back(pose);
  }

  m_BehaviorLog << "generatePoses() ended." << std::endl;
  return poses;
}

RTC::ReturnCode_t ArmImageGenerator::onMoveAutomatic() {
  m_BehaviorLog << "onMoveAutomatic()" << std::endl;

  // ここで目標位置姿勢をリストにして受け取る
  std::vector<RTC::Pose3D> poseArray = generatePoses();

  // ここで繰り返し移動して撮影する
  int count = 0;
  for (auto pose : poseArray) {
    moveAbsWithPose3D(pose);
    saveLog(count++, pose);
  }

  m_BehaviorLog << "onMoveAutomatic() ended." << std::endl;
  return RTC::RTC_OK;
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

  switch (c) {
  case 'a' : // a ならば自動動作をしてonExecuteを返す
    std::cout << "moveAutomatic" << std::endl;
    return onMoveAutomatic();
    break;
  case '0':
    std::cout << "reset" << std::endl;
    moveOrigin();
    break;

  case '1':
    std::cout << "circle" << std::endl;
    break;

  case '2':
    break;

  case 'q':
    std::cout << "Start recording" << std::endl;
    std::cout << "**Reset coordinates" << std::endl;
    targetPos.carPos[0][0] = -1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = 0.25;
    targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = 0;
    targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = -1; targetPos.carPos[2][3] = 0.30;
    ret = m_manipMiddle->movePTPCartesianAbs(targetPos);
    std::cout << ret->id << std::endl;
    if (ret->id != JARA_ARM::OK) { std::cout << " ERRORMESSAGE :" << ret->comment << std::endl; }
    else { std::cout << "OK!" << std::endl; }

    std::cout << " " << std::endl;
    std::cout << "sleep: 1.0" << std::endl;
    coil::sleep(tv);
    std::cout << " " << std::endl;
    targetPos.carPos[0][0] = 1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = 0;
    targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = 0;
    targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = 1; targetPos.carPos[2][3] = 0;


    std::cout << "Layer 1" << std::endl;
    std::cout << "  angle:        15" << std::endl;
    std::cout << "  radius:       7.73cm" << std::endl;
    std::cout << "  height:       2.07cm" << std::endl;

    std::cout << "* move to offset" << std::endl;
    std::cout << "  dir offset:   9" << std::endl;
    targetPos.carPos[0][3] = -0.;  //down
    ret = m_manipMiddle->movePTPCartesianRel(targetPos);
    if (ret->id != JARA_ARM::OK) { std::cout << " ERRORMESSAGE :" << ret->comment << std::endl; }
    else { std::cout << "OK!" << std::endl; }

    std::cout << " " << std::endl;
    std::cout << "sleep: 1.0" << std::endl;
    coil::sleep(tv);
    std::cout << " " << std::endl;
    targetPos.carPos[0][0] = 1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = 0;
    targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = 0;
    targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = 1; targetPos.carPos[2][3] = 0;

    /*
      std::cout << "Layer 2" << std::endl;
      targetPos.carPos[2][3] = -0.0084;  //down
      targetPos.carPos[0][3] = +0.038;   //forward
      targetPos.carPos[0][0] = cos(RADIANS(-25));  //手前
      targetPos.carPos[0][2] = -sin(RADIANS(-25));
      targetPos.carPos[2][0] = sin(RADIANS(-25));
      targetPos.carPos[2][2] = cos(RADIANS(-25));
      ret = m_manipMiddle->movePTPCartesianRel(targetPos);
      if (ret->id != JARA_ARM::OK) { std::cout << " ERRORMESSAGE :" << ret->comment << std::endl; }
      else { std::cout << "OK!" << std::endl; }
    */
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
    m_manipMiddle->setSpeedCartesian(50);
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
  default:
    printf("Unknown Command %c\n", c);
    break;
  }
  /*
    double xlimit[2] = {0.360, 0.400};
    double ylimit[2] = {-185, 185};
    double thlimit[2] = {-M_PI+1.0e-10, M_PI-1.0e-10};


    double x = Uniform() * (xlimit[1] - xlimit[0]) + xlimit[0];
    double y = Uniform() * (ylimit[1] - ylimit[0]) + ylimit[0];
    double th = Uniform() * (thlimit[1] - thlimit[0]) + thlimit[0];

    double z = m_z_prepare_offset + m_z_min; // 40 / 1000.0;
    double z_min = m_z_min; //-72 / 1000.0;

    double s2 = sin(th);
    double c2 = cos(th);

    double x_offset = -m_x_prepare_offset * c2 + m_y_prepare_offset * s2;
    double y_offset =  m_x_prepare_offset * s2 + m_y_prepare_offset * c2;

    std::cout << "--------------------------------------------------" << std::endl;

    JARA_ARM::CarPosWithElbow carPos;

    std::cout << "Reach (" << x << ", " << y << ", " << z << ")" << std::endl;
    carPos.carPos[0][0] = -c2;  carPos.carPos[0][1] = s2; carPos.carPos[0][2] =  0.0; carPos.carPos[0][3] = x + x_offset;
    carPos.carPos[1][0] =  s2;  carPos.carPos[1][1] = c2; carPos.carPos[1][2] =  0.0; carPos.carPos[1][3] = y + y_offset;
    carPos.carPos[2][0] =  0.0; carPos.carPos[2][1] = 0; carPos.carPos[2][2] = -1.0; carPos.carPos[2][3] = z;
    carPos.elbow = 1.0;
    carPos.structFlag = 1;
    JARA_ARM::RETURN_ID_var ret = m_manipMiddle->movePTPCartesianAbs(carPos);
    std::cout << ret->id << std::endl;
    if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    }

    coil::sleep(m_sleepTime);
    std::cout << "[ArmImageGenerator] Down" << std::endl;
    carPos.carPos[2][3] = z_min;
    ret = m_manipMiddle->movePTPCartesianAbs(carPos);
    if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    }

    coil::sleep(m_sleepTime);

    std::cout << "[ArmImageGenerator] Release" << std::endl;
    ret = m_manipMiddle->moveGripper(50);
    if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    }

    coil::sleep(m_sleepTime);

    std::cout << "[ArmImageGenerator] Up" << std::endl;
    carPos.carPos[2][3] = z;
    ret = m_manipMiddle->movePTPCartesianAbs(carPos);
    if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    }

    coil::sleep(m_sleepTime);

    std::cout << "[ArmImageGenerator] Escape" << std::endl;
    //  m_jointPos->length(6);
    m_jointPos[0] = 0;
    m_jointPos[1] = M_PI/4;
    m_jointPos[2] = M_PI/4;
    m_jointPos[3] = 0;
    m_jointPos[4] = M_PI/2;
    m_jointPos[5] = 0;
    ret = m_manipMiddle->movePTPJointAbs(m_jointPos);
    if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    }

    coil::sleep(m_sleepTime);

    JARA_ARM::CarPosWithElbow_var actual(new JARA_ARM::CarPosWithElbow());
    //JARA_ARM::RETURN_ID_var ret2 = m_manipCommon->getFeedbackPosJoint(actual);
    ret = m_manipMiddle->getFeedbackPosCartesian(actual);
    if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    }


    std::cout << "[ArmImageGenerator] Waiting for CameraImage...." << std::ends;

    time_t now = std::time(NULL);
    struct tm* localNow = std::localtime(&now);
    std::ostringstream ss;
    ss
    << "image"
    << 1900 + localNow->tm_year
    << std::setw(2) << std::setfill('0') << localNow->tm_mon + 1
    << std::setw(2) << std::setfill('0') << localNow->tm_mday
    << std::setw(2) << std::setfill('0') << localNow->tm_hour
    << std::setw(2) << std::setfill('0') << localNow->tm_min
    << std::setw(2) << std::setfill('0') << localNow->tm_sec
    << ".png";

    coil::sleep(m_camera_wait_time);

    std::string filename = ss.str();


    #if 1
    /// Capture Image and Save
    bool imageArrived = false;
    //long counter = 0;

    //Inport data check
    while (m_rgbdCameraImageIn.isNew() && (!imageArrived)) {
    m_rgbdCameraImageIn.read();
    imageArrived = true;
    }
    std::cout << "[ArmImageGenerator] Image Arrived." << std::endl;

    long width = m_rgbdCameraImage.data.cameraImage.image.width;
    long height = m_rgbdCameraImage.data.cameraImage.image.height;
    long channels = (m_rgbdCameraImage.data.cameraImage.image.format == Img::CF_GRAY) ? 1 :
    (m_rgbdCameraImage.data.cameraImage.image.format == Img::CF_RGB || m_rgbdCameraImage.data.cameraImage.image.format == Img::CF_PNG || m_rgbdCameraImage.data.cameraImage.image.format == Img::CF_JPEG) ? 3 :
    (m_rgbdCameraImage.data.cameraImage.image.raw_data.length() / width / height);

    if (channels == 3)
    m_buffer.create(height, width, CV_8UC3);

    else
    m_buffer.create(height, width, CV_8UC1);

    long data_length = m_rgbdCameraImage.data.cameraImage.image.raw_data.length();

    //long image_size = width * height * channels;

    if (m_rgbdCameraImage.data.cameraImage.image.format == Img::CF_RGB) {
    for (int i = 0; i<height; ++i)
    memcpy(&m_buffer.data[i*m_buffer.step], &m_rgbdCameraImage.data.cameraImage.image.raw_data[i*width*channels], sizeof(unsigned char)*width*channels);
    if (channels == 3)
    cv::cvtColor(m_buffer, m_buffer, CV_RGB2BGR);
    }
    else if (m_rgbdCameraImage.data.cameraImage.image.format == Img::CF_JPEG || m_rgbdCameraImage.data.cameraImage.image.format == Img::CF_PNG) {
    std::vector<uchar> compressed_image = std::vector<uchar>(data_length);
    memcpy(&compressed_image[0], &m_rgbdCameraImage.data.cameraImage.image.raw_data[0], sizeof(unsigned char) * data_length);

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

    cv::imwrite(m_logDir + "/" + filename, m_buffer);

    #endif

    m_JointLog << x << ", " << y << ", " << th << ", " << filename << ", depth_" << filename << std::endl;

    long d_width = m_rgbdCameraImage.data.depthImage.width;
    long d_height = m_rgbdCameraImage.data.depthImage.height;
    long size = d_width * d_height;


    m_DepthLog << "[" ;
    for(int i=0; i<size; ++i){
    m_DepthLog << m_rgbdCameraImage.data.depthImage.raw_data[i] << ",";
    }
    m_DepthLog << "]" << std::endl;


    std::cout << "[ArmImageGenerator] Ready" << std::endl;
    m_jointPos[0] = 0;
    m_jointPos[1] = M_PI/4;
    m_jointPos[2] = M_PI/4;
    m_jointPos[3] = 0;
    m_jointPos[4] = M_PI/2;
    m_jointPos[5] = 0;
    m_manipMiddle->movePTPJointAbs(m_jointPos);
    coil::sleep(m_sleepTime);

    std::cout << "[ArmImageGenerator] Reach" << std::endl;
    carPos.carPos[2][3] = z;
    ret = m_manipMiddle->movePTPCartesianAbs(carPos);
    if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    }

    coil::sleep(m_sleepTime);

    std::cout << "[ArmImageGenerator] Down" << std::endl;
    carPos.carPos[2][3] = z_min;
    ret = m_manipMiddle->movePTPCartesianAbs(carPos);
    if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    }

    coil::sleep(m_sleepTime);

    std::cout << "[ArmImageGenerator] Hold" << std::endl;

    double ratio = m_gripper_close_ratio > 1.0 ? 1.0 : m_gripper_close_ratio < 0.0 ? 0 : m_gripper_close_ratio;
    ret = m_manipMiddle->moveGripper(100 * ratio);//m_gripper_close_ratio);
    if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    }

    //ret1 = m_manipMiddle->moveGripper(10);
    coil::sleep(m_sleepTime);

    std::cout << "[ArmImageGenerator] Up" << std::endl;
    carPos.carPos[2][3] = z;
    ret = m_manipMiddle->movePTPCartesianAbs(carPos);
    if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    }

    coil::sleep(m_sleepTime);

    std::cout << "[ArmImageGenerator] Ready" << std::endl;
    m_jointPos[0] = 0;
    m_jointPos[1] = M_PI/4;
    m_jointPos[2] = M_PI/4;
    m_jointPos[3] = 0;
    m_jointPos[4] = M_PI/2;
    m_jointPos[5] = 0;
    ret = m_manipMiddle->movePTPJointAbs(m_jointPos);
    if (ret->id != JARA_ARM::OK) {
    std::cout << "ERROR in ServoON" << std::endl;
    std::cout << " ERRORCODE    :" << ret->id << std::endl;
    std::cout << " ERRORMESSAGE :" << ret->comment << std::endl;
    }

    coil::sleep(m_sleepTime);
    std::cout << "------------------------------------------------------------" << std::endl;

    m_JointLog.flush();
  */

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


