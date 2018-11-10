#include <ros/ros.h>
#include <iostream>
#include <mavros_msgs/StreamRate.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <fstream>

#define PI 3.14159265


float pose_px,pose_py,pose_qw,pose_qz;
geometry_msgs::TwistStamped YawVel;
Eigen::Vector3f euler;
float alpha;
bool poseflag=false;
float err_ang=5,ua;
float d_err,err_prev=0;
float i_err=0;
float kp_ang=0.01;
float ki_ang=0;
float kd_ang=0.00003;
float tha = 0.6;
bool controlOrientation = false;


void poseGPScb(const geometry_msgs::PoseStamped& msg){

  pose_qw = msg.pose.orientation.w;
  pose_qz = msg.pose.orientation.z;
  pose_py = msg.pose.position.y;
  pose_px = msg.pose.position.x;

  euler= Eigen::Quaternionf(pose_qw,0, 0, pose_qz).toRotationMatrix().eulerAngles(0, 1, 2);
  alpha = euler[2];
  poseflag = true;
}

void YawControlRad(float ref_abs);

std::ofstream expGiro;


int main(int argc, char **argv) {
  int angREF;

  expGiro.open("expGiro.txt");

  float yaw_ref;

  if (argc == 3) {
    yaw_ref = std::atof(argv[1]);
    angREF  = std::atoi(argv[2]);
  }else{
    ROS_ERROR("GIMME yaw angle in radians and REF value");
    return -1;
  }

  mavros_msgs::PositionTarget move_drone;

  move_drone.coordinate_frame = 8;
  move_drone.type_mask = 4039;//2503

  int rate = 60;

  ros::init(argc, argv, "wp_follow");
  ros::NodeHandle n;

  ros::Rate r(rate);

  ros::Subscriber odom_ = n.subscribe("/mavros/local_position/pose",1,poseGPScb);
  ros::Publisher move_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",1);
  ros::Publisher move_Yaw = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);

  while (poseflag==false) { //wait for poseCB
    r.sleep();
    ros::spinOnce();
  }

  float ref_abs = alpha + yaw_ref;//fixed

  ros::Time currentT = ros::Time::now();
  ros::Time prev=currentT;
  float time_delta=0;
  double acc=0;

  float  diff;

  while (ros::ok()) {
    /* code */



    YawControlRad(ref_abs);
    YawVel.header.stamp =  ros::Time::now();
    move_Yaw.publish(YawVel);
    time_delta = currentT.toSec()-prev.toSec();
    acc= acc+time_delta;
    diff =(angREF - err_ang)*PI/180;
    
    expGiro << acc << ","<< atan2(sin(diff),cos(diff))*180/PI << "," << angREF << "\n"; 
    prev = currentT;
    currentT = ros::Time::now();
    ros::spinOnce();
    r.sleep();

    if(controlOrientation){//&&controlPosition){
      std::cout << "Llegué"<< std::endl;
      break;
    }

  }

  return 0;
}


void YawControlRad(float ref_abs){

  err_ang = (ref_abs - alpha);
  err_ang = atan2(sin(err_ang),cos(err_ang))*180/PI;

  d_err = err_ang-err_prev;
  err_prev = err_ang;
  i_err+=err_ang;

  ua = kp_ang*err_ang  +  kd_ang*d_err  +  ki_ang*i_err;

  if(ua>=tha){
    ua=tha;
  }else if(ua<=-tha){
    ua=-tha;
  }

  if((err_ang<0.1)&&(err_ang>-0.1)){
    ua = 0.0;
    controlOrientation = true;
  }
  YawVel.twist.angular.z=ua;

}
