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

std::ofstream myfile;
std::ofstream traj;
  double acc=0;
  double dREF=20.0;

  ros::Time prevGlobal;
  ros::Time currentGlobal;

float ang_medido;


int main(int argc, char** argv){

  if(argc == 3){
    ref_x = std::atof(argv[1]);
    ref_y = std::atof(argv[2]);
  }else if (argc == 2) {
    /* code */
    yaw_ref = std::atof(argv[1]);
  }else{

      ROS_ERROR("GIMME X Y");
      return -1;
  }


  myfile.open("expRRT2.txt",std::ofstream::out | std::ofstream::app);
  traj.open("tRRT2.txt",std::ofstream::out | std::ofstream::app);



  U_control.twist.linear.x=0;
  U_control.twist.linear.y=0;
  U_control.twist.linear.z=0;
  U_control.twist.angular.x=0;
  U_control.twist.angular.y=0;
  U_control.twist.angular.z=0;

  int rate = 60;

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

  yaw_ref = atan2((ref_y-pose_py),(ref_x-pose_px));
  yaw_ref = atan2(sin(yaw_ref),cos(yaw_ref));

  ref_abs = (alpha + yaw_ref);//fixed

  ref_abs = atan2(sin(ref_abs),cos(ref_abs));

  std::cout << "yaw_r ef" << yaw_ref*180/PI << " alpha " << alpha*180/PI << std::endl;
    get_d_err(ref_x,ref_y);
    get_a_err(ref_x,ref_y);




  while(ros::ok()){



/*
    if((err_dist>err_distTH)){
      controlOrientation = false;
      YawControl(ref_x,ref_y);

    }else{
      U_control.twist.angular.z=0;
      controlOrientation = true;
    }

    if((err_ang<err_angTH)&&(err_ang>-err_angTH)){
      controlPosition = false;
      PositionControl(ref_x,ref_y);
      std::cout << "distance" << '\n';
      std::cout << "E_angular: " << err_ang<< '\n';
      controlOrientation = true;

    }else{
      U_control.twist.linear.x =0;
      U_control.twist.linear.y =0;
      controlPosition = true;
    }

*/

    get_d_err(ref_x,ref_y);
    get_a_err(ref_x,ref_y);
    time_delta = currentT.toSec()-prev.toSec();

    ang_medido =  atan2(sin(yaw_ref-err_ang*PI/180),cos(yaw_ref-err_ang*PI/180));


    acc= acc+time_delta;
    //std::cout << "time: " << acc << std::endl;
    std::cout << "REF: " << yaw_ref*180/PI << "---MEAS: " << ang_medido*180/PI << std::endl;
    myfile << acc  << "," << pose_px << "," << ref_x << "," << pose_py <<","<<ref_y<<","<< (ang_medido)*180/PI << ","<< yaw_ref*180/PI << "\n";
    lastv = err_ang;
    traj << pose_px << "," << pose_py << "\n";
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
        //std::cout << "YEP" <<std::endl;
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

  

  ros::Time finish = ros::Time::now();

  while( (ros::Time::now().toSec()-finish.toSec()) < 5){
    get_d_err(ref_x,ref_y);
    get_a_err(ref_x,ref_y);
    acc= acc+currentT.toSec()-prev.toSec();
    std::cout << "time: " << acc << std::endl;
        ang_medido =  atan2(sin(yaw_ref-err_ang*PI/180),cos(yaw_ref-err_ang*PI/180));

    myfile << acc  << "," << pose_px << "," << ref_x << "," << pose_py <<","<<ref_y<<","<< ang_medido*180/PI << ","<< yaw_ref*180/PI << "\n";
    traj << pose_px << "," << pose_py << "\n";
    prev = currentT;
    currentT = ros::Time::now();
    ros::spinOnce();
    r.sleep();
  }



  myfile.close();
  traj.close();
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

    ang_medido =  atan2(sin(yaw_ref-err_ang*PI/180),cos(yaw_ref-err_ang*PI/180));


    myfile << acc  << "," << pose_px << "," << ref_x << "," << pose_py <<","<<ref_y<<","<< ang_medido*180/PI << ","<< yaw_ref*180/PI << "\n";
    std::cout << "x " << U_control.twist.linear.x << "  y : " <<  U_control.twist.linear.y <<std::endl;
    traj << pose_px << "," << pose_py << "\n";

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

  std::cout << "ANGULO: "<< alpha*180/PI << std::endl;

  poseflag = true;
}
 