// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  private Swerve drivetrain;
  private String LLName;

  public Limelight(String LLName, Swerve drivetrain) {
    this.LLName = LLName;
    this.drivetrain = drivetrain;
  }

  public Translation2d getTranslationToTag() {
    // Pose2d pose = LimelightHelpers.getBotPose2d(LLName);
    return new Translation2d();
  }
  public void getTarget() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
