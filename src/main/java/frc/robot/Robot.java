// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.Optional;

/**
 * The main robot class. This handles the robot container (which contains the
 * subsystems) and handles triggers during different modes.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Transform2d positionalError = new Transform2d();

  private Transform2d limelightOffset = new Transform2d();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Optional<Pose2d> limelightPose = m_robotContainer.getLimelightInterface().determinePosition()
        .map(currentPose -> currentPose.toPose2d());
    Pose2d swervePose = m_robotContainer.getDrivetrain().getPose();

    Optional<Double> limelightTargetDistance = m_robotContainer.getLimelightInterface().calculateDistanceToTarget();
    if (limelightPose.isPresent() && limelightTargetDistance.isPresent() && limelightTargetDistance.get() < Constants.Vision.FrontLimelight.maximumPoseCalculationDistance) {
      limelightOffset = limelightPose.get().minus(swervePose);
    }

    m_robotContainer.getPlayingField().setRobotPose(swervePose.plus(limelightOffset));
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    System.out.println("re enabled");
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.getDrivetrain().resetToAbsEncoders();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
