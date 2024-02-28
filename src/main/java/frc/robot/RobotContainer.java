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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 *  commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController driveController = new CommandXboxController(Constants.kDriverControllerPort);

  private final Swerve drivetrain = new Swerve();
  private final Limelight limelight = new Limelight();
  private final Pneumatics pneumatics = new Pneumatics();

  private final IntakeIndexer intakeIndexer = new IntakeIndexer();
  private final Launcher launcher = new Launcher(limelight);

  private final IntakeCommand intakeCommand = new IntakeCommand(intakeIndexer);
  
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            () -> -driveController.getLeftY(), // Translation
            () -> -driveController.getLeftX(), // Strafe
            () -> -driveController.getRightX(), // Rotation
            () -> driveController.leftBumper().getAsBoolean() // Field-oriented driving (yes or no)
        ));

    CameraServer.startAutomaticCapture();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    driveController.a().onTrue(new InstantCommand(() -> {
      drivetrain.zeroGyro();
    }, drivetrain));


    driveController.y().onTrue(
      new InstantCommand(() -> {
        pneumatics.toggleClimber();
      }, pneumatics)
    );
    
    driveController.x().onTrue(
      new InstantCommand(() -> {
        pneumatics.toggleAmpGuide();
      }, pneumatics)
    );
    
    driveController.leftBumper().toggleOnTrue(intakeCommand);
    driveController.rightBumper().onTrue(new Shoot(launcher, intakeIndexer, () -> driveController.getRightTriggerAxis()));
    driveController.leftTrigger().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> intakeIndexer.startSpittingNote()),
      new WaitCommand(1),
      new InstantCommand(() -> intakeIndexer.stop())
      ));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Limelight getLimelight() {
    return limelight;
  }
}
