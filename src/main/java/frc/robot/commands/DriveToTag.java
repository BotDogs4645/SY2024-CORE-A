// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AprilTag;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToTag extends Command {
  /** Creates a new DriveToTag. */

  private Swerve drivetrain;
  private Pose3d target;

  private TrajectoryConfig config = 
    new TrajectoryConfig(
      Constants.Swerve.maxSpeed,
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
    )
    .setKinematics(Constants.Swerve.swerveKinematics);

  public DriveToTag(Swerve drivetrain, Pose3d target) {
    this.drivetrain = drivetrain;
    this.target = target;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,new Rotation2d(0)),
      List.of(),
      target.toPose2d(),
      config
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
