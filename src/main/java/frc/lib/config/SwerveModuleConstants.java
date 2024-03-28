package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;
  public final Rotation2d angleOffset;
  public final boolean driveIsInverted;
  public final int contCurrentLimit;
  // public final double currentLimitAmperes;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   * 
   * @param angleOffset the offset that is subtracted (not added) from angle
   *                    measurements in the relevant Swerve module
   */
  public SwerveModuleConstants(
      int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset, boolean driveIsInverted, int contCurrentLimit) {
    this.driveIsInverted = driveIsInverted;
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = canCoderID;
    this.angleOffset = angleOffset;
    this.contCurrentLimit = contCurrentLimit;    
  }
}
