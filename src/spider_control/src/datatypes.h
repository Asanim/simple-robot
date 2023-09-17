#ifndef DATATYPES_H
#define DATATYPES_H

#include <Servo.h>
#include <math.h>
#include <math.h>

extern Servo servo[4][3];
extern volatile float site_now[4][3];
extern volatile float site_expect[4][3];
extern float temp_speed[4][3];
extern volatile int rest_counter;
extern const float length_a;
extern const float length_b;
extern const float length_c;
extern const float length_side;
extern const float z_absolute;
extern const float z_default;
extern const float z_up;
extern const float z_boot;
extern const float x_default;
extern const float x_offset;
extern const float y_start;
extern const float y_step;
extern const float y_default;
extern float move_speed;
extern float speed_multiple;
extern const float spot_turn_speed;
extern const float leg_move_speed;
extern const float body_move_speed;
extern const float stand_seat_speed;
extern const float KEEP;
extern const float pi;
extern const float temp_a;
extern const float temp_b;
extern const float temp_c;
extern const float temp_alpha;
extern const float turn_x1;
extern const float turn_y1;
extern const float turn_x0;
extern const float turn_y0;

#endif // DATATYPES_H
