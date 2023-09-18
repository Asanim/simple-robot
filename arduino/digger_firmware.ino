
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <servo.h> 

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

//drive (some may be redundant...
const int del = 100;
const float dead_zone = 0.1;

ros::NodeHandle  nh;

//servos (pan+tilt only for now)
const int InitialState[12] = {90,50}; //,0,0,0,0,0,0,0,0,0,0}; //{90,90,90,90,90,90,90,90,90,90,90,90};
Servo servo[2];
//pins from 22-34


float float2dp (float foo) {
  return float(int (foo * 100))/100;
}

void motor_cb(const geometry_msgs::Twist& msg){
  //normalize teleop input
  float vel_lin = msg.linear.x/0.22;
  float vel_ang = msg.angular.z/2.84;
      
  if (vel_lin >= dead_zone) {
      forward();
  } else if (vel_lin <= -dead_zone) {
    back();
  } else {
    stop_all();
  }
  //bad code i know!
  //if linear velocity=0, go directly to linear
  if (vel_lin !=0) {
    delay(del*(vel_lin/(vel_lin+vel_ang)));
  }

  if (vel_ang >= dead_zone) {
    left();
  } else if (vel_ang <= -dead_zone) {
    right();
  } else {
    stop_all();
  }
  //bad code i know!
  //if angular velocity=0, linear command will continue
  if (vel_ang !=0) {
    delay(del*(vel_ang/(vel_lin+vel_ang)));
  }
}

//as long as we get input, move forward
void test_cb(const geometry_msgs::Twist& msg){
  forward();
}

//void servo_cb(const sensor_msgs::JointState& cmd_msg){

//todo: no array min size checks...
void servo_cb(const std_msgs::Int16MultiArray& cmd_msg){

  //26 to 34
  for (int i=0;i<8;i++) {
    if (cmd_msg.data[i] > 0) {
      digitalWrite(i+26, HIGH);
    } else {
      digitalWrite(i+26, LOW);
    }
  }
  
  //tilt = 34
  servo[0].write(cmd_msg.data[8]);
  //pan = 35
  servo[1].write(cmd_msg.data[9]);
}


ros::Subscriber<geometry_msgs::Twist> sub_twist("cmd_vel", motor_cb);
//ros::Subscriber<sensor_msgs::JointState> sub_servo("digger_arm", servo_cb);
ros::Subscriber<std_msgs::Int16MultiArray> sub_servo("digger_arm", servo_cb);

void setup() {
  // Setup mega outputs
  for (int i=22;i<=33;i++) {
    pinMode(i, OUTPUT); 
    digitalWrite(i, LOW);
  }
  
  //setup Servos (pan/tilt)
  for (int i=0;i<2;i++) {
    pinMode(i+34, OUTPUT);
    servo[i].attach(i+34);
    
    //inital state index: pin34=index0
    servo[i].write(InitialState[i]);
  } 

  //ROS init
  nh.initNode();
  nh.subscribe(sub_twist); 
  nh.subscribe(sub_servo); 
}

int testdelay = 200;

void test_arm () {
  for (int i=26;i<=34;i++) {
    digitalWrite(i, HIGH);
    digitalWrite(i-1, LOW);
    delay(100);
  }
}

void test_drive () {
  forward();
  delay(1000);
  back();
  delay(1000);
  left();
  delay(1000);
  right();
  delay(1000);
}

void forward () {
  digitalWrite(22, HIGH);
  digitalWrite(23, LOW);
  digitalWrite(24, LOW);
  digitalWrite(25, HIGH);
}

void stop_all () {
  digitalWrite(22, LOW);
  digitalWrite(23, LOW);
  digitalWrite(24, LOW);
  digitalWrite(25, LOW);
}

void back() {
  digitalWrite(22, LOW);
  digitalWrite(23, HIGH);
  digitalWrite(24, HIGH);
  digitalWrite(25, LOW);
}

void left () {
  digitalWrite(22, HIGH);
  digitalWrite(23, LOW);
  digitalWrite(24, HIGH);
  digitalWrite(25, LOW);
}

void right () {
  digitalWrite(22, LOW);
  digitalWrite(23, HIGH);
  digitalWrite(24, LOW);
  digitalWrite(25, HIGH);
}

void loop() {
  nh.spinOnce();
  delay(1);
}