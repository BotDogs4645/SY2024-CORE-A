// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * The main robot class. This handles the robot container (which contains the
 * subsystems) and handles triggers during different modes.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Optional<Pose2d> estimatedNotePose = m_robotContainer.getBackLimelight().getEstimatedTargetPose();
    if (estimatedNotePose.isPresent()) {
      System.out.println("Estimated 'note' offset (x, y): (" + estimatedNotePose.get().getX() + ", " + estimatedNotePose.get().getY() + ")");

      System.out.println("Estimated Limelight pitch: " + m_robotContainer.getDistanceEstimation().calculateLimelightPitch(1, m_robotContainer.getBackLimelight().getTargetInformation().get()[1]));
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    System.out.println("Re-enabled");
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.getDrivetrain().resetToAbsEncoders();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
