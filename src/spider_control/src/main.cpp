/* -----------------------------------------------------------------------------
  - Project: Remote control Crawling robot
  - Author:  panerqiang@sunfounder.com
  - Date:  2015/1/27
  - Edited by: Asanim
   -----------------------------------------------------------------------------
  - Overview
  - This project was written for the Crawling robot desigened by Sunfounder.
    This version of the robot has 4 legs, and each leg is driven by 3 servos.
  This robot is driven by a Ardunio Nano Board with an expansion Board.
  We recommend that you view the product documentation before using.
  - Request
  - This project requires some library files, which you can find in the head of
    this file. Make sure you have installed these files.
  - How to
  - Before use,you must to adjust the robot,in order to make it more accurate.
    - Adjustment operation
    1.uncomment ADJUST, make and run
    2.comment ADJUST, uncomment VERIFY
    3.measure real sites and set to real_site[4][3], make and run
    4.comment VERIFY, make and run
  The document describes in detail how to operate.
   ---------------------------------------------------------------------------*/

// modified by Regis for spider project

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include "utils.h"
#include "servo_handler.h"
#include "control.h"
#include "datatypes.h"
#include "state_publisher.h"

/* Servos --------------------------------------------------------------------*/
//define 12 servos for 4 legs
Servo servo[4][3];

/* Size of the robot ---------------------------------------------------------*/
const float length_a = 55;
const float length_b = 77.5;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;
/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;
const float y_default = x_default;

/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];    //real-time coordinates of the end of each leg
volatile float site_expect[4][3]; //expected coordinates of the end of each leg
float temp_speed[4][3];   //each axis' speed, needs to be recalculated before each movement
float move_speed;     //movement speed
float speed_multiple = 1; //movement speed multiple
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;      //+1/0.02s, for automatic rest
//functions' parameter
const float KEEP = 255;
//define PI for calculation
const float pi = 3.1415926;
/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;
/* ---------------------------------------------------------------------------*/

/*
  - setup function
   ---------------------------------------------------------------------------*/
void setup()
{
  std::cout << "Robot starts initialization" << std::endl;
  //initialize default parameter
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  //start servo service
  std::thread servo_thread(servo_service);
  servo_thread.detach();
  std::cout << "Servo service started" << std::endl;
  //initialize servos
  servo_attach();
  std::cout << "Servos initialized" << std::endl;
  std::cout << "Robot initialization Complete" << std::endl;
}

/*
  - loop function
   ---------------------------------------------------------------------------*/
void loop()
{
  std::cout << "Stand" << std::endl;
  stand();
  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::cout << "Step forward" << std::endl;
  step_forward(20);
  std::this_thread::sleep_for(std::chrono::seconds(2));

//   init_servo();
//   std::this_thread::sleep_for(std::chrono::seconds(2));
//   std::cout << "Step back" << std::endl;
//   step_back(5);
//   std::this_thread::sleep_for(std::chrono::seconds(2));
//   std::cout << "Turn left" << std::endl;
//   turn_left(5);
//   std::this_thread::sleep_for(std::chrono::seconds(2));
//   std::cout << "Turn right" << std::endl;
//   turn_right(5);
//   std::this_thread::sleep_for(std::chrono::seconds(2));
//   std::cout << "Hand wave" << std::endl;
//   hand_wave(3);
//   std::this_thread::sleep_for(std::chrono::seconds(2));
//   std::cout << "Hand wave" << std::endl;
//   hand_shake(3);
//   std::this_thread::sleep_for(std::chrono::seconds(2));  
//   std::cout << "Sit" << std::endl;
//   sit();
//   std::this_thread::sleep_for(std::chrono::seconds(5));
}

int main()
{
  setup();
  
  // Start StatePublisher in a separate thread
  std::thread state_publisher_thread([]() {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<StatePublisher>(servo);
    rclcpp::spin(node);
    rclcpp::shutdown();
  });
  state_publisher_thread.detach();

  while (true)
  {
    loop();
  }
  return 0;
}