package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class OnboardModuleState {

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing in
   * appropriate scope for CTRE and REV onboard control as both controllers as of writing don't have
   * support for continuous input.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();

    // Rotating the wheel by 180 degrees and reversing the direction results in
    // essentially no net change to the rotation and direction of the wheel spin
    // This is used here to reduce wheel rotation.
    if (Math.abs(delta) > 90) {
      targetSpeed *= -1;
      targetAngle += delta > 0 ? -180 : 180;
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * Returns an angle congruent to {@code newAngle} that is as close as possible
   * to {@code scopeReference}.
   * <br>
   * For example, given a reference of 360 and an angle of 1, this function will
   * return 361, as 361 is the closest angle to the reference that is congruent
   * to the reference.
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    // Figure out how many revolutions from the angle to the reference
    double diffRevs = Math.round((scopeReference - newAngle) / 360) * 360;

    // Add that many revolutions
    return diffRevs + newAngle;
  }
}
