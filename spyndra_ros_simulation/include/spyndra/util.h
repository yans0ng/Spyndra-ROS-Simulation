/* util.h
 *
 * Utility functions for Spyndra hardware.
 */

#ifndef SPYNDRA_UTIL_H
#define SPYNDRA_UTIL_H
namespace spyndra
{
const double PI = 3.1415926;
/* cmd_to_rad
 * Maps command of servo motors to joint angles in radians
 */
double cmd_to_rad( int leg, double pwm )
{
  if (leg % 2 == 0) // Femur mapping function
    return (pwm - 255.0) * PI / 450.0;
  else              // Tibia mapping function
    return (90.0 - 9.0/23.0 * ( pwm - 260.0 )) * PI / 180;

}
}

#endif
