#ifndef CONTROL_H
#define CONTROL_H

#include "datatypes.h"

/**
 * @brief Turns the robot to the left.
 * 
 * @param step Number of steps to turn.
 */
void turn_left(unsigned int step);

/**
 * @brief Turns the robot to the right.
 * 
 * @param step Number of steps to turn.
 */
void turn_right(unsigned int step);

/**
 * @brief Moves the robot forward.
 * 
 * @param step Number of steps to move.
 */
void step_forward(unsigned int step);

/**
 * @brief Moves the robot backward.
 * 
 * @param step Number of steps to move.
 */
void step_back(unsigned int step);

/**
 * @brief Moves the robot's body to the left.
 * 
 * @param i Steps to move.
 */
void body_left(int i);

/**
 * @brief Moves the robot's body to the right.
 * 
 * @param i Steps to move.
 */
void body_right(int i);

/**
 * @brief Robot wave one leg.
 * 
 * @param i Number of waves.
 */
void hand_wave(int i);

/**
 * @brief Robot shake one leg.
 * 
 * @param i Number of shakes.
 */
void hand_shake(int i);

/**
 * @brief Sets the state for a single joint.
 * 
 * @param leg Leg index.
 * @param x X coordinate.
 * @param y Y coordinate.
 * @param z Z coordinate.
 */
void set_site(int leg, float x, float y, float z);

/**
 * @brief Waits for one of the joints to reach the set state.
 * 
 * @param leg Leg index.
 */
void wait_reach(int leg);

/**
 * @brief Waits for all joints to reach their set state.
 */
void wait_all_reach(void);

/**
 * @brief Robot sit down.
 */
void sit(void);

/**
 * @brief Robot stand up.
 */
void stand(void);

#endif // CONTROL_H
