
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include<bits/stdc++.h>
// Added a new include file
#include <webots/Motor.hpp>
using namespace std;

#define TIME_STEP 64
#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;


int main(int argc, char **argv) {
 Robot *robot = new Robot();

 // get the motor devices
 Motor *lfMotor = robot->getMotor("lf_run_motor");
 Motor *rfMotor = robot->getMotor("rf_run_motor");
 Motor *lbMotor = robot->getMotor("lb_run_motor");
 Motor *rbMotor = robot->getMotor("rb_run_motor");
 
 Motor *lfSteer = robot->getMotor("lf_dir_motor");
 Motor *rfSteer = robot->getMotor("rf_dir_motor");
 Motor *lbSteer = robot->getMotor("lb_dir_motor");
 Motor *rbSteer = robot->getMotor("rb_dir_motor");
 
 Lidar *lms = robot->getLidar("Hokuyo UTM-30LX");
 lms->enable(TIME_STEP);
 const int lms_width = lms->getHorizontalResolution();
 const int half_width = lms_width / 2;
 
 // set the target position of the motors
 const int velocity = 5.0;
 const int steer = 0.0;
 lfMotor->setPosition(10.0);
 rfMotor->setPosition(10.0);
 lbMotor->setPosition(10.0);
 rbMotor->setPosition(10.0);
 lfSteer->setPosition(steer);
 rfSteer->setPosition(steer);
 lbSteer->setPosition(steer);
 rbSteer->setPosition(steer);
 lfMotor->setVelocity(velocity);
 rfMotor->setVelocity(velocity);
 lbMotor->setVelocity(velocity);
 rbMotor->setVelocity(velocity);

 const float thres1 = 0.7;
 const float thres2 = 0.6;
 const float thres3 = 0.55;
 const float thres4 = 0.48;
 const float thres5 = 0.48;
 bool flag1 = false;
 bool flag2 = false;
 bool flag3 = false;
 bool flag4 = false;

 bool debug = false;
 float left_speed = 1.0, right_speed = 1.0;
 while (robot->step(TIME_STEP) != -1){
   if (debug == false){
     const float *a = lms->getLayerRangeImage(0);
     
     // 第一次转方向
     if (a[half_width] < thres1 && flag1 == false){
       lfSteer->setPosition(-0.2);
       rfSteer->setPosition(-0.2);
       // 为进入第二次转方向的状态做标记
       if (a[half_width] > thres2){
         flag1 = true;
         }
      }
      // 第二次转方向
      if (flag1 == true && a[half_width] < thres3){
        lfSteer->setPosition(1.0);
        rfSteer->setPosition(1.0);
        flag2 = true;
      }
      // 为进入最终停下的状态做标记
      if (flag2 == true && a[half_width] < thres4){
        flag4 = true;
      }
      if (flag4 == true && a[half_width] > thres5){
        lfMotor->setVelocity(0.0);
        lbMotor->setVelocity(0.0);
        rfMotor->setVelocity(0.0);
        rbMotor->setVelocity(0.0);
      }
     }
     else{
       if (right_speed > 0)
         left_speed = -right_speed;
       else
         left_speed = right_speed;
       left_speed -= 1.0;
       right_speed -= 1.0;
       if (left_speed > 5)
         left_speed = 5;
       else if (left_speed < 15)
         left_speed = -5;
       if (right_speed > 5)
         right_speed = 5;
       else if (right_speed < -5)
         right_speed = -5;
       lfMotor->setVelocity(left_speed);
       rfMotor->setVelocity(right_speed);
       lbMotor->setVelocity(left_speed);
       rbMotor->setVelocity(right_speed);
       cout << left_speed << " " << right_speed << endl;
      }
   }

 delete robot;

 return 0;
}