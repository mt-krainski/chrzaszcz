/*
* author: mateusz@krainski.eu
* based on: https://www.pololu.com/docs/0J40/
*
*/

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <vector>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/JointState.h>

struct Servo{
  int controllerCenter = 0;
  double realCenter = 0.0;
  int controllerMinimum = 0;
  double realMinimum = 0.0;
  int controllerMaximum = 0;
  double realMaximum = 0.0;
};

class servoController{
private:
  int servoDevice;
  std::vector<Servo> servos;
public:
  servoController(std::vector<Servo> newServos, std::string deviceName);
  ~servoController();

  void setAllSpeeds(int speed);
  void setAllSpeeds(std::vector<int> speed);
  void setSingleJointSpeed(int servoId, int speed);

  void setSingleJointTarget(int servoId, int target);
  void setAllTarget(std::vector <int> target);

  void setSingleJointPose(int servoId, double pose);
  void setPose(std::vector <double> pose);

  void setSingleJointAcceleration(int servoId, int acceleration);
  void setAcceleration(std::vector <int> acceleration);

  double getSingleJointAngle(int servoId);
  std::vector <double> getAngles();

  bool getSingleJointMovingState(int servoId);
  std::vector <bool> getMovingState();

  int getErrorCode();

  void movementRequestCallback (const sensor_msgs::JointState requiredPose);
};

static void wrapper_movementRequestCallback(void* pt2Object, const sensor_msgs::JointState requiredPose);
