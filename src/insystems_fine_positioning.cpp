#include "node_pid.h"
#include <math.h>
#include <toolkit_cpp/math_tools.h>
#include <toolkit_cpp/param_tools.h>
#include <toolkit_cpp/data_converter.h>
#include <toolkit_cpp/tf_tools.h>
#include <toolkit_cpp/point_2d.h>
#include <toolkit_cpp/line_segment_2d.h>
#include <toolkit_cpp/sim_tools.h>
#include <angles/angles.h>
#include <boost/thread.hpp>
#include <unistd.h>
#include "dem_pid/FinePos.h"
#include "dem_pid/FinePosF.h"
#include <insystems_msgs/RobotHardwareStatus.h>
#include <toolkit_cpp/param_tools.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>

using toolkit_cpp::Point2d;

#define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher.
//#define TOLERANCE 0.378  // Distance from the target distance, at which the distance will be considered as achieved. TRIANGLE/office
//#define TOLERANCE 0.125  //showroom docking station
//#define TOLERANCE 0.338 //loading_station
//#define TOLERANCE 0.34  //old coveyor
#define TOLERANCE 0.6
//#define TOLERANCE 0.25 //CAMERA
#define TOLERANCE_ANGLE 0.01  // Differenc from target angle, which will be tolerated.
#define MAX_SPEED 0.1    // Maximum speed of robot.
#define MAX_A_SPEED 0.5    // Maximum angular speed of robot.
#define PUBLISHER_TOPIC "/yocs_cmd_vel_mux/fine_pos/cmd_vel"
#define SUBSCRIBER_TOPIC "odom"
#define PI 3.141592
#define F_KP 2.58  // P constant for PSD translation controller
#define F_KD 0.047  // D constant for PSD translation controller
#define F_KI 0.0  // S constant for PSD translation controller
#define R_KP 2.0  // P constant for PSD rotation controller
#define R_KD 0.1  // D constant for PSD rotation controller
#define R_KI 0.0  // S constant for PSD rotation controller

boost::recursive_mutex mutex_;

double angle_to_target=0, distance_to_target=0;
double old_angle_to_target=0;
double angle_to_path=0, distance_to_path=0;
double old_angle_to_path=0;

double diff_x = 0;
double diff_y = 0;
double diff_z = 0;

double target_x = 0;
double target_y = 0;
double target_z = 0;

double old_target_x = 0;
double old_target_y = 0;
double old_target_z = 0;

MyPoint *start;     // Start position. Distance will be measured from here
MyPoint *last;      // Last position of robot.
double tolerance;   // Tolerated deviation from target distance.
double maxSpeed;    // Maximal velocity.
double maxASpeed;   // Maximal angular velocity.
ros::Publisher pubMessage; // Object for publishing messages.
int iterations;        // Number of received messages.
double sumDistance;    // Sum of distance errors for PSD controller.
double sumAngle;       // Sum of angle errors for PSD controller.
double toleranceAngle; // Tolerated deviation from target angle.
int callbacks=0;
double targetDistance=0, targetAngle=0;
double final_angle = 0;
double final_distance = 0;

bool Start = false;
bool in_charge = false;
bool axis_finished = false;
bool straight = false;
bool finished = false;
bool inside = false;

double _tolerance;
double _distLaserCenter;

double test_target_q = 0;
double roll, pitch, yaw;
double yaw_degrees;

LineSegment2d poseToLineSegment(const geometry_msgs::PoseStamped::ConstPtr& pose) {

    //ROS_DEBUG("FinePositioning::poseToLineSegment");

    double x1 = pose->pose.position.x;
    double y1 = pose->pose.position.y;

    geometry_msgs::Quaternion quat = pose->pose.orientation;
    double yaw = tf::getYaw(quat);

    //ROS_INFO("Yaw : %f", yaw);

    Point2d source_point(x1, y1);
    Point2d target_point = Point2d::fromPolarCoordinates(1.0, yaw);
    target_point.add(Point2d(x1, y1));

    LineSegment2d line_segment(source_point, target_point);
    return line_segment;
}


double calculatePSD(MyPoint* actual, double actualValue, double lastValue, double reference, double kP, double kD, double kS, double *sum)
{
  double speed = 0;
  double error = reference - actualValue;
  double previousError = reference - lastValue;
  double dt = actual->time.toSec() - last->time.toSec();
  double derivative = (error - previousError)/dt;
  *sum = *sum + error*dt;
  speed = kP*error + kD*derivative + kS*(*sum);
  return speed;
}

bool closeEnough(MyPoint* actual)
{
  double distance;
  distance = start->getDistance(actual)*copysign(1.0, final_distance);
  if (fabs(distance-final_distance) > tolerance)
  {
    return false;
  }
  if (fabs(final_angle - (actual->angle - start->angle)) > toleranceAngle &
    fabs(final_angle - (actual->angle - start->angle) + 2*PI) > toleranceAngle &
    fabs(final_angle - (actual->angle - start->angle) - 2*PI) > toleranceAngle)
  {
    return false;
  }
  return true;
}

void publishMessage(double angleCommand, double speedCommand)
{
  //preparing message
  geometry_msgs::Twist msg;

  msg.linear.x = speedCommand;
  if (angleCommand >= 0.5){
     msg.angular.z = angleCommand + 0.5;
  } else {
     msg.angular.z = angleCommand;  
  }
  
  //publishing message
  pubMessage.publish(msg);
}


void TargetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  int it = 0;
  if (Start){
  //boost::recursive_mutex::scoped_lock lock(mutex_);
  
  
  ROS_INFO("In Target Callback");
   
  target_x = msg->pose.position.x;
  target_y = msg->pose.position.y;

  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);

  yaw = angles::normalize_angle_positive(yaw) - PI;
  yaw_degrees = yaw * (180.0/3.14);

  //0.31 blue AGV
  Point2d reference_point_2d = Point2d(-_distLaserCenter, 0.0);
  //Point2d reference_point_2d = Point2d(-0.38, 0.0);

  LineSegment2d pose_as_line = poseToLineSegment(msg);
  Point2d projected_point_on_line = pose_as_line.projectPointOnLine(reference_point_2d);

  distance_to_path = reference_point_2d.getDistanceTo(projected_point_on_line);
  angle_to_path = reference_point_2d.getCenterDirection(projected_point_on_line);
 
  angle_to_target = reference_point_2d.getCenterDirection(Point2d(target_x, target_y));
  distance_to_target = reference_point_2d.getDistanceTo(Point2d(target_x, target_y));

   if (straight == false){
      tolerance = 0.1;
      toleranceAngle = 0.1;
      final_angle = yaw;
      final_distance = 0;
   }
   else {
      if (axis_finished == false){
         tolerance = 0.1;
         toleranceAngle = 0.1;
         final_angle = angle_to_path;
         final_distance = distance_to_path;
      } else {
         tolerance = _tolerance;
         toleranceAngle = TOLERANCE_ANGLE;
         final_angle = angle_to_target;
         final_distance = distance_to_target;
         //final_distance = distance_to_path;
   }
  }
  double angleCommand = 0;
  double speedCommand = 0;

  MyPoint* actual = new MyPoint(0.0, -_distLaserCenter, 2.0*asin(0.0), msg->header.stamp);

  if (closeEnough(actual) == true)
  {
    callbacks++;
    
    if (callbacks == 1) {
    
        ROS_INFO("straight");
        publishMessage(0.0,0.0);
        straight = true;
    
    }
    
    else if (callbacks == 2){

        ROS_INFO("Axis reached");
        publishMessage(0.0,0.0);      
        axis_finished = true ;

        if (angle_to_target > 0)
        { 
              double turn90_time = fabs((PI/2) / maxASpeed);

              geometry_msgs::Twist msg;
              //ROS_INFO("turn_time : %f", turn_time);

              msg.linear.x = 0.0;
              msg.angular.z = maxASpeed;
  
              double end;
              double start; 

              start = end = ros::Time::now().toSec();
              double p = end - start;

              ros::Rate loop_rate(10);
              while (p < (turn90_time)){
                    pubMessage.publish(msg);
                    end = ros::Time::now().toSec();
                    p = end - start;
                    loop_rate.sleep();
                    //ROS_INFO("Turn_Time iteration");
               } 
 
               msg.linear.x = 0.0;
               msg.angular.z = 0.0;
               pubMessage.publish(msg);      
        
         } else if (angle_to_target < 0)
         {
               double turn90_time = fabs((PI/2) / maxASpeed);

              geometry_msgs::Twist msg;
              //ROS_INFO("turn_time : %f", turn_time);

              msg.linear.x = 0.0;
              msg.angular.z = -maxASpeed;
  
              double end;
              double start; 

              start = end = ros::Time::now().toSec();
              double p = end - start;

              ros::Rate loop_rate(10);
              while (p < (turn90_time)){
                    pubMessage.publish(msg);
                    end = ros::Time::now().toSec();
                    p = end - start;
                    loop_rate.sleep();
                    //ROS_INFO("Turn_Time iteration");
               } 
 
               msg.linear.x = 0.0;
               msg.angular.z = 0.0;
               pubMessage.publish(msg); 
         }
 
    } else {
        ROS_INFO("Target reached");
        publishMessage(0.0,0.0);
        Start = false;
        finished = true;
       axis_finished = false;
       callbacks = 0;
    }
    
  }
  if (iterations == 0)
  {
    start->x = actual->x;
    start->y = actual->y;
    start->time = actual->time;
    start->angle = actual->angle;
    last->x = actual->x;
    last->y = actual->y;
    last->time = actual->time;
    last->angle = actual->angle;
  }
  iterations++;

  //Calculation of action intervention.
  if (fabs(final_distance) > tolerance)
  {
    speedCommand = calculatePSD(actual,start->getDistance(actual)*copysign(1.0, final_distance),start->getDistance(last)*copysign(1.0, final_distance),final_distance,F_KP,F_KD,F_KI,&sumDistance);
  }

  if (actual->angle-last->angle < -PI)
  {
    actual->angle += 2*PI;
  } 
  else if (actual->angle-last->angle > PI)
  {
    actual->angle -= 2*PI;
  }

  angleCommand = calculatePSD(actual,actual->angle-start->angle, last->angle-start->angle,final_angle,R_KP,R_KD,R_KI,&sumAngle);

  //Saving position to last
  last->x = actual->x;
  last->y = actual->y;
  last->time = actual->time;
  last->angle = actual->angle;

  //Invoking method for publishing message
  publishMessage(fmin(maxASpeed,angleCommand), fmin(maxSpeed,speedCommand)); 
}

}

bool startfp(dem_pid::FinePos::Request  &req, dem_pid::FinePos::Response &res)
{ 
  if (req.Start == true){
     if (Start == false){
	   ROS_INFO("FINE_POS STARTED");
           res.Started = true;
           Start = true;
	   finished = false;
	}
     else {
           res.Started = false;
           }
  }else {
	ROS_INFO("FINE_POS STOPPED");
	Start = false;
	finished = false;
	straight = false;
	axis_finished = false;
	res.Started = false;
  }
  return true;
}

bool finishfp(dem_pid::FinePosF::Request &req, dem_pid::FinePosF::Response &res)
{
	if (req.Check == true){
		res.Finished = finished;
	}
	return true;
}

void hardwareCallback(const insystems_msgs::RobotHardwareStatus::ConstPtr& msg)
{
    in_charge = msg->in_charge;
}

void onToleranceChanged(double& param) {
    _tolerance = param;
}

void onDistChanged(double& param) {
    _distLaserCenter = param;
}

int main(int argc, char **argv)
{ 
 
  //Initialization of node
  ros::init(argc, argv, "insystems_fine_positioning_test");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  ParamTools::getAndSubscribe(private_nh, "tolerance", _tolerance, onToleranceChanged);
  ParamTools::getAndSubscribe(private_nh, "distLaserCenter", _distLaserCenter, onDistChanged);

  ros::ServiceServer servicefp = n.advertiseService("fine_pos", startfp);
  ros::ServiceServer servicefpf = n.advertiseService("fine_pos_finished", finishfp);

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC,1);
  ros::Subscriber hardware_sub = n.subscribe("hardware_status", 1000, hardwareCallback);
  ros::Subscriber subTarget = n.subscribe("target_pose", 1000, TargetCallback);

  maxSpeed = MAX_SPEED;
  maxASpeed = MAX_A_SPEED;  
  pubMessage = pub;
  iterations = 0;
  sumDistance = 0;
  sumAngle = 0;
  start = new MyPoint(0.0, 0.0, 0.0, ros::Time::now());
  last = new MyPoint(0.0, 0.0, 0.0, ros::Time::now());

  ros::spin();

  return 0;
}
