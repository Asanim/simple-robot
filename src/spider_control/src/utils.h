#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include "datatypes.h"

/**
 * @brief Transforms Cartesian coordinates to polar coordinates.
 * 
 * @param alpha alpha angle.
 * @param beta beta angle.
 * @param gamma gamma angle.
 * @param x X coordinate.
 * @param y Y coordinate.
 * @param z Z coordinate.
 */
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z);

/**
 * @brief Transforms polar coordinates to servo angles.
 * 
 * @param leg Leg index.
 * @param alpha Alpha angle.
 * @param beta Beta angle.
 * @param gamma Gamma angle.
 */
void polar_to_servo(int leg, float alpha, float beta, float gamma);

#endif // UTILS_H
