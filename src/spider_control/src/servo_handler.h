#ifndef SERVO_HANDLER_H
#define SERVO_HANDLER_H

#include "datatypes.h"

const int servo_pin[4][3] = { {22, 23, 24}, {25, 26, 27}, {28, 29, 30}, {31, 32, 33} };

/**
 * @brief Timer interrupt function to set each servo state on time.
 */
void servo_service(void);

/**
 * @brief Maps servos to pin numbering. 
 */
void servo_attach(void);

/**
 * @brief Detaches the servos from each pwm pin.
 */
void servo_detach(void);

/**
 * @brief Initializes all servos/joints to default standing.
 */
void init_servo(void);

#endif // SERVO_HANDLER_H
