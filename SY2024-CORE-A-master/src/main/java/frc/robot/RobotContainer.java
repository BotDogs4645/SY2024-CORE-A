// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.AprilTag;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController driveController = new CommandXboxController(0);

  private final Swerve drivetrain = new Swerve();
  private final AprilTag aprilTagInstance = new AprilTag();
  private final Pneumatics m_pneumaticsSubsystem = new Pneumatics();
  private final CommandXboxController m_driverController = new CommandXboxController(Constants.kDriverControllerPort);
  
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            () -> -driveController.getLeftY(), // tanslation
            () -> -driveController.getLeftX(), // strafe
            () -> -driveController.getRightX(), // rotation
            () -> driveController.leftBumper().getAsBoolean() // feild oriented yes or no
        ));


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    m_driverController.b().onTrue(m_pneumaticsSubsystem.toggleClimber());
    driveController.a().onTrue(new InstantCommand(() -> {
      drivetrain.zeroGyro();
    }, drivetrain));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public AprilTag getLimelight() {
    return aprilTagInstance;
  }
}
