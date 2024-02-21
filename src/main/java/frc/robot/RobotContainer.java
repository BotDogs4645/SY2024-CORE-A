// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Limelight;
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

  private final CommandXboxController driveController = new CommandXboxController(Constants.kDriverControllerPort);

  private final Swerve drivetrain = new Swerve();
  private final Limelight limelight = new Limelight();
  
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
    driveController.a().onTrue(new InstantCommand(() -> {
      drivetrain.zeroGyro();
    }, drivetrain));

    // driveController.y().onTrue(new DriveToTag(
    //   drivetrain,
    //   limelight.getTargetPose()
    // ));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Limelight getLimelight() {
    return limelight;
  }
}
