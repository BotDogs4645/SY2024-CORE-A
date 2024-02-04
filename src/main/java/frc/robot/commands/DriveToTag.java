// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AprilTag;
import frc.robot.subsystems.Swerve;

public class DriveToTag extends Command {
  /** Creates a new DriveToTag. */

  Swerve drivetrain;
  AprilTag tag;
  Pose3d offset;

  Pose3d finalPose;
  Pose3d target;

  public DriveToTag(Swerve drivetrain, AprilTag tag, Pose3d offset) {
    this.drivetrain = drivetrain;
    this.tag = tag;
    this.offset = offset;
  }

  public DriveToTag(Swerve drivetrain, Pose3d target) {
    this.drivetrain = drivetrain;
    this.target = target;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Pose3d targetingData = tag.getTagPose();
    // finalPose = new Pose3d(
    //   targetingData.getX() - offset.getX(),
    //   targetingData.getY() - offset.getY(),
    //   targetingData.getZ() - offset.getZ(),
    //   targetingData.getRotation().minus(offset.getRotation())
    // );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrain.driveToTag(finalPose);
    drivetrain.driveToTag(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
