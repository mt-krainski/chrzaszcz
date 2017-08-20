
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>

#include "servoController.h"

static void wrapper_movementRequestCallback(void* pt2Object, const sensor_msgs::JointState requiredPose)
{
  servoController* sc = (servoController*) pt2Object;
  sc->movementRequestCallback(requiredPose);
}

typedef const boost::function< void(const sensor_msgs::JointState &)>  callback;

int main(int argc, char** argv) {

  std::vector<Servo> servos;
  const int controllerCenters[] = {2300, 2300, 1500, 1500, 1500, 1400};
  const double realCenters[] = {1.0/35.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const int controllerLowerServoLimits[] = {700, 900, 1000, 1000, 1000, 950};
  const double realLowerServoLimits[] = {0.0, -135.0*M_PI/180.0, -45.0*M_PI/180.0, -45.0*M_PI/180.0, -45.0*M_PI/180.0, -45.0*M_PI/180.0};
  const int controllerHigherServoLimits[] = {2300, 2300, 2000, 2000, 2000, 1800};
  const double realHigherServoLimits[] = {1.0/35.0, 0.0*M_PI/180.0, 45.0*M_PI/180.0, 45.0*M_PI/180.0, 45.0*M_PI/180.0, 45.0*M_PI/180.0};

  for(size_t i=0; i<6; i++)
  {
    servos.push_back(Servo());
    servos[i].controllerCenter = controllerCenters[i];
    servos[i].realCenter = realCenters[i];
    servos[i].controllerMinimum = controllerLowerServoLimits[i];
    servos[i].realMinimum = realLowerServoLimits[i];
    servos[i].controllerMaximum = controllerHigherServoLimits[i];
    servos[i].realMaximum = realHigherServoLimits[i];
  }

  servoController sc(servos, "/dev/ttyACM0");

  std::vector<int> zeros (controllerCenters, controllerCenters + sizeof(controllerCenters) / sizeof(controllerCenters[0]) );
  std::vector<double> home =  {1.0/35.0, 0.0, -70.0*M_PI/180.0, 70.0*M_PI/180.0, 70.0*M_PI/180.0, 0.0};

  sc.setAllSpeeds(20);
  sc.setPose(home);
  // sc.setAllTarget(zeros);

  auto returnedAngles = sc.getAngles();

  callback servoControllerCallback = boost::bind(&wrapper_movementRequestCallback, &sc, _1);

  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
  ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("requested_pose", 1000, servoControllerCallback);
  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(30);

  sensor_msgs::JointState joint_state;

  while (ros::ok()) {
      returnedAngles = sc.getAngles();
      // ROS_INFO_STREAM("looping....");
      // ROS_INFO_STREAM(returnedAngles[0]<< " "<< returnedAngles[1]<< " "<< returnedAngles[3]<< " ");
      //update joint_state
      joint_state.header.stamp = ros::Time::now();
      joint_state.name.resize(7);
      joint_state.position.resize(7);
      joint_state.name[5] ="first_to_base";
      joint_state.position[5] = returnedAngles[5];
      joint_state.name[4] ="second_to_first";
      joint_state.position[4] = -returnedAngles[4];
      joint_state.name[3] ="third_to_second";
      joint_state.position[3] = returnedAngles[3];
      joint_state.name[2] ="fourth_to_third";
      joint_state.position[2] = -returnedAngles[2];
      joint_state.name[1] ="fifth_to_fourth";
      joint_state.position[1] = returnedAngles[1];
      joint_state.name[0] ="right_gripper_to_fifth_link";
      joint_state.position[0] = returnedAngles[0];
      joint_state.name[6] ="left_gripper_to_fifth_link";
      joint_state.position[6] = -returnedAngles[0];

      //send the joint state and transform
      joint_pub.publish(joint_state);

      // This will adjust as needed per iteration
      ros::spinOnce();
      loop_rate.sleep();
  }


  return 0;
}
