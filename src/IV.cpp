/*

IV5



*/


#include <ros/ros.h>
#include <iostream>
#include <mavros_msgs/StreamRate.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <fstream>

//#include <Eigen/Dense>
bool poseflag=false;

bool controlPosition = false;
bool controlOrientation = false;

#define PI 3.14159265


//control functions
void YawControl(float x, float y),PositionControl(float x, float y);
void YawControlRad(void);

//YawControlVariables

geometry_msgs::TwistStamped U_control;



float pose_px,pose_py,pose_qw,pose_qz;
float err_ang=100,ua,ud;
float d_err,err_prev=0;
float i_err=0;
float i_err_acc=0;
float kp_ang=0.01;
float ki_ang=0.00002;//0.00003; //0.00003;
float kd_ang=0.002;//0.007;//0.007;

//PositionControlVariables
float kp_dist=0.5;
float ki_dist=0.0;//0.00001;
float kd_dist=0.01;//0.00005;
float errx,erry;
float d_errd=0;
float err_prevd=0;
float int_error=0;
float int_acc=0;

float ref_x;
float ref_y;

float yaw_ref;
float ref_abs;

float err_dist=5;

Eigen::Vector3f euler;

float thd = 0.6;
float tha = 0.6;

float alpha;
float ang_medido;

//YawControlCallbacks
void poseGPScb(const geometry_msgs::PoseStamped& msg);
void get_d_err(float x, float y);
void get_a_err(float x, float y);
void softBreak(ros::Rate& r, ros::Publisher& pub_control);
void hardBreak(ros::Rate& r, ros::Publisher& pub_control);

float err_angTH = 5;
float err_distTH = 0.75;
  float time_delta=0;
  float lastv=0;
  float lastv_ref=0;
float lastx,lasty;
std::ofstream myfile;
  double acc=0;
  double dREF=20.0;

  ros::Time prevGlobal;
  ros::Time currentGlobal;

int  ct=0;


 ros::Time finish;



int main(int argc, char** argv){
std::vector<float> xx,yy,aa;

xx.push_back(0);
yy.push_back(10);

xx.push_back(10);
yy.push_back(10);

xx.push_back(10);
yy.push_back(0);

xx.push_back(0);
yy.push_back(0);

  myfile.open("Exp1.txt");



  U_control.twist.linear.x=0;
  U_control.twist.linear.y=0;
  U_control.twist.linear.z=0;
  U_control.twist.angular.x=0;
  U_control.twist.angular.y=0;
  U_control.twist.angular.z=0;

  int rate = 10;

  ros::init(argc, argv, "wp_follow");
  ros::NodeHandle n;

  ros::Rate r(rate);

  ros::Subscriber odom_ = n.subscribe("/mavros/local_position/pose",1,poseGPScb);

  ros::Publisher pub_control = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);


  ros::Time currentT = ros::Time::now();
  ros::Time prev=currentT;



  while (poseflag==false) { //wait for poseCB
    r.sleep();
    ros::spinOnce();
  }

  flag1:

	std::cout << "woppi"<< std::endl;






if(ct>3){
	std::cout << "hasfdasf"<< std::endl;
	goto flag2;
}
ref_x = xx[ct];
ref_y = yy[ct]; 
ct++;

  ref_abs = alpha + yaw_ref;//fixed
    get_d_err(ref_x,ref_y);
    get_a_err(ref_x,ref_y);




  while(ros::ok()){

    get_d_err(ref_x,ref_y);
    get_a_err(ref_x,ref_y);
    time_delta = currentT.toSec()-prev.toSec();

    acc= acc+time_delta;
    std::cout << "time: " << acc << std::endl;
    ang_medido =  atan2(sin(ref_abs-err_ang*PI/180),cos(ref_abs-err_ang*PI/180));
    //    time stamp || pose_x || ref_x || pose_y || ref_y ||ang || angREF
    myfile << acc  << "," << pose_px <<","<< ref_x << ","<< pose_py << "," << ref_y <<","<< ang_medido*180/PI << "," << ref_abs*180/PI << "\n";
    lastv = err_ang;
    lastx = ref_x;
    lasty = ref_y;
    lastv_ref = ref_abs;
    prev = currentT;
    currentT = ros::Time::now();

    if((err_dist>err_distTH)){
      controlOrientation = false;
      YawControl(ref_x,ref_y);

      if((err_ang<err_angTH)&&(err_ang>-err_angTH)){
        controlPosition = false;
        PositionControl(ref_x,ref_y);
      }else{

        U_control.twist.linear.x =0;
        U_control.twist.linear.y =0;
      }

    }else{



      currentGlobal = currentT;
      prevGlobal = prev;
      softBreak(r,pub_control);
      currentT = currentGlobal;
      prev = prevGlobal;
      controlOrientation = true;
      controlPosition = true;
    }

    U_control.header.stamp = ros::Time::now();
    pub_control.publish(U_control);

    ros::spinOnce();
    r.sleep();

    if(controlOrientation&&controlPosition){//&&controlPosition){
      std::cout << "LLegué "<< std::endl;
      break;
    }
  }

  goto flag1;

  flag2:

  finish = ros::Time::now();

  while( (ros::Time::now().toSec()-finish.toSec()) < 5){
    get_d_err(ref_x,ref_y);
    get_a_err(ref_x,ref_y);
    acc= acc+currentT.toSec()-prev.toSec();
    std::cout << "time: " << acc << std::endl;
    ang_medido =  atan2(sin(ref_abs-err_ang*PI/180),cos(ref_abs-err_ang*PI/180));
    myfile << acc  << "," << pose_px <<","<< lastx << ","<< pose_py << "," << lasty <<","<< ang_medido*180/PI << "," << ref_abs*180/PI << "\n";
    prev = currentT;
    currentT = ros::Time::now();
    ros::spinOnce();
    r.sleep();
  }

  


  myfile.close();
  return 0;
}

void softBreak(ros::Rate& r, ros::Publisher& pub_control){
  int stepB = 4;
  float xB = U_control.twist.linear.x/float(stepB);
  float yB = U_control.twist.linear.y/float(stepB);

  if(xB==0&&yB==0){

    U_control.twist.linear.x = 0.4*cos(alpha);
    U_control.twist.linear.y = 0.4*sin(alpha);
    xB = U_control.twist.linear.x/float(stepB);
    yB = U_control.twist.linear.y/float(stepB);

  }


  for(int i=0;i<(stepB);i++){
    get_d_err(ref_x,ref_y);
    get_a_err(ref_x,ref_y);

    prevGlobal = currentGlobal;
    currentGlobal = ros::Time::now();

    time_delta = currentGlobal.toSec()-prevGlobal.toSec();
    acc= acc+time_delta;
    ang_medido =  atan2(sin(ref_abs-lastv*PI/180),cos(ref_abs-lastv*PI/180));

    myfile << acc  << "," << pose_px << "," << ref_x << "," << pose_py << "," << ref_y << "," << ang_medido*180/PI << "," << lastv_ref*180/PI  << "\n";
    
    std::cout << "x " << U_control.twist.linear.x << "  y : " <<  U_control.twist.linear.y <<std::endl;


    U_control.twist.angular.z=0.0;


    U_control.twist.linear.x -=xB;
    U_control.twist.linear.y -=yB;


    U_control.header.stamp = ros::Time::now();
    pub_control.publish(U_control);
    ros::spinOnce();
    r.sleep();

  }

  hardBreak(r,pub_control);
}

void hardBreak(ros::Rate& r, ros::Publisher& pub_control){
  U_control.twist.angular.z=0.0;
  U_control.twist.linear.x =0.0;
  U_control.twist.linear.y =0.0;
  U_control.header.stamp = ros::Time::now();
  pub_control.publish(U_control);
  ros::spinOnce();
  r.sleep();
}

void get_d_err(float x, float y){
  erry = y - pose_py;
  errx = x - pose_px;

  err_dist = sqrt(pow(erry,2)+pow(errx,2));
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

  U_control.twist.angular.z=ua;
}


/*
void YawControlRad(void){

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

  if((err_ang<err_angTH)&&(err_ang>-err_angTH)){
    ua = 0.0;
    controlOrientation = true;
  }
  YawVel.twist.angular.z=ua;

}
*/

void PositionControl(float x, float y){
  erry = y - pose_py;
  errx = x - pose_px;

  err_dist = sqrt(pow(erry,2)+pow(errx,2));

  d_errd = err_dist - err_prevd;
  err_prevd = err_dist;
  int_error = (err_dist+err_prevd)*(time_delta/2);
  int_acc += int_error;

  ud = kp_dist*err_dist + kd_dist*d_errd + ki_dist*int_acc;

  if(ud<=0.0){
    ud = 0.0;
  }else if(ud>=thd){
    ud = thd;
  }

 /* if(err_dist<err_distTH){
    ud = 0.0;
    controlPosition = true;
  }*/
  U_control.twist.linear.x = ud*cos(alpha);
  U_control.twist.linear.y = ud*sin(alpha);
}

void poseGPScb(const geometry_msgs::PoseStamped& msg){

  pose_qw = msg.pose.orientation.w;
  pose_qz = msg.pose.orientation.z;
  pose_py = msg.pose.position.y;
  pose_px = msg.pose.position.x;

  euler= Eigen::Quaternionf(pose_qw,0, 0, pose_qz).toRotationMatrix().eulerAngles(0, 1, 2);
  alpha = euler[2];//-PI/2;

  alpha = atan2(sin(alpha),cos(alpha));
  poseflag = true;
}
