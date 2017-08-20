
#include "servoController.h"


servoController::servoController(std::vector<Servo> newServos, std::string deviceName)
{
  servos = newServos;
  servoDevice = open(deviceName.c_str(), O_RDWR | O_NOCTTY);

  termios options;
  tcgetattr(servoDevice, &options);
  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options.c_oflag &= ~(ONLCR | OCRNL);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tcsetattr(servoDevice, TCSANOW, &options);
  // return 0;
}

servoController::~servoController()
{
  close(servoDevice);
}

void servoController::setAllSpeeds(int speed)
{
  for(size_t i=0; i<6; i++)
    setSingleJointSpeed(i, speed);
}

void servoController::setAllSpeeds(std::vector<int> speeds)
{
  for(size_t i=0; (i<speeds.size()) && (i<6); i++)
    setSingleJointSpeed(i, speeds[i]);
}

void servoController::setSingleJointSpeed(int servoId, int speed)
{
  unsigned char speedlow = (speed & 0x7F);
  unsigned char speedhigh = (speed >> 7) & 0x7F;
  unsigned char chan = servoId & 0x7F;
  // std::string data;
  unsigned char data[] = {0xAA, 0x0c, 0x07, chan, speedlow, speedhigh};
  write(servoDevice, data, sizeof(data));
}

void servoController::setSingleJointTarget(int servoId, int target)
{
  target *= 4;
  // position = position * 4
  unsigned char poslo = (target & 0x7f);
  unsigned char poshi = (target >> 7) & 0x7f;
  unsigned char chan  = servoId &0x7f;
  unsigned char data[] = {0xAA, 0x0c, 0x04, chan, poslo, poshi};
  write(servoDevice, data, sizeof(data));
}
void servoController::setAllTarget(std::vector <int> target)
{
  for(size_t i=0; (i<target.size()) && (i<6); i++)
    setSingleJointTarget(i, target[i]);
}

void servoController::setSingleJointPose(int servoId, double pose)
{
  if(pose > 2*servos[servoId].realMaximum)
    pose = 2*servos[servoId].realMaximum;
  else if(pose < 2*servos[servoId].realMinimum)
    pose = 2*servos[servoId].realMinimum;

  const double angleRelative = (pose - servos[servoId].realMinimum)/(servos[servoId].realMaximum - servos[servoId].realMinimum);
  const int newTarget = int(angleRelative*(servos[servoId].controllerMaximum - servos[servoId].controllerMinimum) + servos[servoId].controllerMinimum);
  setSingleJointTarget(servoId, newTarget);
}

void servoController::setPose(std::vector <double> pose)
{
  for(size_t i=0; (i<pose.size()) && (i<6); i++)
    setSingleJointPose(i, pose[i]);
}

void servoController::setSingleJointAcceleration(int servoId, int acceleration)
{
 //TODO implement
}
void servoController::setAcceleration(std::vector <int> acceleration)
{
//TODO implement
}

double servoController::getSingleJointAngle(int servoId)
{
  unsigned char chan  = servoId &0x7f;
  unsigned char data[] =  {0xaa, 0x0c, 0x10, chan};
  write(servoDevice, data, sizeof(data));
  unsigned char response[2];
  boost::this_thread::sleep(boost::posix_time::milliseconds(2));
  if (read(servoDevice, response, 2) != 2)
  {
    perror("error reading");
    return -1;
  }
  double controllerPose = static_cast<double>(response[0]) + static_cast<double>(response[1]<<8);
  double poseRelative = (controllerPose/4.0 - servos[servoId].controllerMinimum)/(servos[servoId].controllerMaximum - servos[servoId].controllerMinimum);
  return poseRelative*(servos[servoId].realMaximum - servos[servoId].realMinimum) + servos[servoId].realMinimum;
}

std::vector <double> servoController::getAngles()
{
  std::vector<double> angles;
  for(size_t i=0; i<6; i++)
    angles.push_back(getSingleJointAngle(i));

  return angles;
}

bool servoController::getSingleJointMovingState(int servoId)
{
//TODO implement
}

std::vector <bool> servoController::getMovingState()
{
//TODO implement
}

int servoController::getErrorCode()
{
//TODO implement
}

void servoController::movementRequestCallback (const sensor_msgs::JointState requiredPose)
{
  std::vector<double> newPose;
  newPose.resize(6);
  for(size_t i=0; i<requiredPose.name.size(); i++)
  {
    if(requiredPose.name[i] == "right_gripper_to_fifth_link")
      newPose[0] = requiredPose.position[i];
    else if(requiredPose.name[i] == "fifth_to_fourth")
      newPose[1] = requiredPose.position[i];
    else if(requiredPose.name[i] == "fourth_to_third")
      newPose[2] = requiredPose.position[i];
    else if(requiredPose.name[i] == "third_to_second")
      newPose[3] = requiredPose.position[i];
    else if(requiredPose.name[i] == "second_to_first")
      newPose[4] = requiredPose.position[i];
    else if(requiredPose.name[i] == "first_to_base")
      newPose[5] = requiredPose.position[i];
  }

  setPose(newPose);
}
