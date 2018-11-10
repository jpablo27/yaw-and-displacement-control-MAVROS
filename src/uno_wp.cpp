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

  float time_delta=0;
void poseGPScb(const geometry_msgs::PoseStamped& msg){

  pose_qw = msg.pose.orientation.w;
  pose_qz = msg.pose.orientation.z;
  pose_py = msg.pose.position.y;
  pose_px = msg.pose.position.x;

  euler= Eigen::Quaternionf(pose_qw,0, 0, pose_qz).toRotationMatrix().eulerAngles(0, 1, 2);
  alpha = euler[2];
  poseflag = true;
}

void YawControl(float x, float y);
void get_a_err(float x, float y);

std::ofstream expGiro;
std::ofstream traj;

float ang_medido;

int main(int argc, char **argv) {
  int angREF;

  expGiro.open("expRRT2.txt",std::ofstream::out | std::ofstream::app);
  traj.open("tRRT2.txt",std::ofstream::out | std::ofstream::app);

  float yaw_ref;
  float wpx,wpy;

  if (argc == 3) {
    wpx = std::atof(argv[1]);
    wpy = std::atoi(argv[2]);
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


  ros::Time currentT = ros::Time::now();
  ros::Time prev=currentT;

  double acc=0;

  float  diff;

 

  get_a_err(wpx,wpy);

  yaw_ref = atan2((wpy-pose_py),(wpx-pose_px));
  yaw_ref = atan2(sin(yaw_ref),cos(yaw_ref));

  while (ros::ok()) {
    /* code */
    get_a_err(wpx,wpy);
    ang_medido =  atan2(sin(yaw_ref-err_ang*PI/180),cos(yaw_ref-err_ang*PI/180));

    
    expGiro << acc << ","<< pose_px << "," <<"1080"<< "," << pose_py << "," << "1080" << ","<< ang_medido*180/PI << "," << yaw_ref*180/PI  << "\n";
    traj << pose_px << "," << pose_py << "\n";
    std::cout << "REF: " << yaw_ref*180/PI << "---MEAS: " << ang_medido*180/PI << std::endl;


    YawControl(wpx,wpy);
    YawVel.header.stamp =  ros::Time::now();
    move_Yaw.publish(YawVel);
    time_delta = currentT.toSec()-prev.toSec();
    acc= acc+time_delta;
    diff =(angREF - err_ang)*PI/180;

 
    prev = currentT;
    currentT = ros::Time::now();
    ros::spinOnce();
    r.sleep();

    if(controlOrientation){//&&controlPosition){
      std::cout << "Llegué"<< std::endl;
      break;
    }

  }

  expGiro.close();
  traj.close();

  return 0;
}

void get_a_err(float x, float y){
  err_ang = (atan2((y-pose_py),(x-pose_px)) - euler[2]);
  err_ang = atan2(sin(err_ang),cos(err_ang))*180/PI;
}

void YawControl(float x, float y){
  err_ang = (atan2((y-pose_py),(x-pose_px)) - euler[2]);
  err_ang = atan2(sin(err_ang),cos(err_ang))*180/PI;

  d_err = (err_ang-err_prev)/time_delta;
  err_prev = err_ang;
  i_err+=(err_ang+err_prev)*(time_delta/2);

  ua = kp_ang*err_ang  +  kd_ang*d_err  +  ki_ang*i_err;

  if(ua>=tha){
    ua=tha;
  }else if(ua<=-tha){
    ua=-tha;
  }

  if((err_ang<1)&&(err_ang>-1)){
    ua = 0.0;
    controlOrientation = true;
  }

  YawVel.twist.angular.z=ua;
}