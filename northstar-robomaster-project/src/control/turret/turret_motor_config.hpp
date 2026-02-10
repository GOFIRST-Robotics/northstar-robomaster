#ifndef TURRET_MOTOR_CONFIG_HPP_
#define TURRET_MOTOR_CONFIG_HPP_

#include <cassert>
#include <cstdint>

namespace src::control::turret
{
/**
 * Configuration struct for the TurretMotor object.
 */
struct TurretMotorConfig
{
    float startAngle = 0;  /// Angle (in radians) where the turret is assumed to start at. This
                           /// angle value maps to the same value (in encoder ticks) as
                           /// startEncoderValue.
    uint16_t startEncoderValue = 0;  /// Encoder value between [0, ENC_RESOLUTION) associated
                                     /// with startAngle.
    float minAngle = 0;  /// Min angle that the turret will be limited to (in radians). DOES NOT
                         /// have to be wrapped between [0, 2 PI), but must be > maxAngle.
    float maxAngle = 0;  /// Max angle that the turret will be limited to (in radians). This
                         /// value should be > minAngle.
    bool limitMotorAngles = true;  /// true if the motor will be limited between [minAngle,
                                   /// maxAngle], false otherwise.
    float ratio = 1; /*1 / number of rotatons needed to turn moving pivot one ratation*/
};
}  // namespace src::control::turret

#endif  // TURRET_MOTOR_CONFIG_HPP_
