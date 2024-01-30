// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.synth.SynthTextAreaUI;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.config.CTREConfigs;
import frc.robot.subsystems.Limelight;
import frc.lib.util.AprilTag;

/**
 * The main robot class. This handles the robot container (which contains the
 * subsystems) and handles triggers during different modes.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Limelight limelight;
  private AprilTag aprilTag;

  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    m_robotContainer = new RobotContainer();
    limelight = new Limelight();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    var t = limelight.targetPos();
    // Transform3d relativeRobotPosition;
    if (t == null) {
      System.out.println("No Limelight target.");
    } else {
      // System.out.printf("Target position: {x: %.3f, y: %.3f, z: %.3f}\n", t.getX(), t.getY(), t.getZ());

      // relativeRobotPosition = Limelight.determineRelativePosition();

      // double currentAprilTagHeight = Constants.APRILTAGS.get(Limelight.entry("tid").getInteger(-1)).getZ();

      if (AprilTag.getDirectDistance() != -1) {
        System.out.println("Current spatial distance to target: " + AprilTag.getDirectDistance());
        // System.out.println("Limelight Angular Data: " + LimelightHelpers.getTX(""));
        // System.out.println("Limelight 'tx' data: " + Limelight.entry("tx").getDouble(-1));
        // System.out.println("Limelight 'ty' data: " + Limelight.entry("ty").getDouble(-1));

        // System.out.println("Current distance to primary target: " + );
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

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