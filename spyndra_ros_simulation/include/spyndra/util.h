/* util.h
 *
 * Utility functions for Spyndra hardware.
 */

namespace spyndra
{
/* cmd_to_rad
 * Maps command of servo motors to joint angles in radians
 */
double cmd_to_rad( int leg, double cmd )
{
  if (leg % 2 == 0) // Femur mapping function
    return (cmd - 300.0) / 150.0 * M_PI /4.0 + 0.2;
  else              // Tibia mapping function
    return (350.0 - cmd ) / 150.0 * M_PI /4.0 + 0.2;
}
}
