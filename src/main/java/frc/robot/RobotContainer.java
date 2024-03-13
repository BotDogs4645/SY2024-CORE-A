// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PathPlanner;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.IntakeFromSource;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.BackLimelight;
import frc.robot.subsystems.FrontLimelight;
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
  private final CommandXboxController manipulatorController = new CommandXboxController(Constants.kManipulatorControllerPort);

  private final Swerve drivetrain = new Swerve();
  private final FrontLimelight frontLimelight = new FrontLimelight();
  private final BackLimelight backLimelight = new BackLimelight();
  private final Pneumatics pneumatics = new Pneumatics();
  private final IntakeIndexer intakeIndexer = new IntakeIndexer();
  private final Shooter shooter = new Shooter();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            () -> -driveController.getLeftY(), // Translation
            () -> -driveController.getLeftX(), // Strafe
            () -> -driveController.getRightX(), // Rotation
            () -> driveController.rightTrigger().getAsBoolean() // Field-oriented driving (yes or no)
        ));

    CameraServer.startAutomaticCapture();

    autoChooser = AutoBuilder.buildAutoChooser();

    NamedCommands.registerCommand("Shoot Speaker", new ShootSpeaker(intakeIndexer, shooter));
    NamedCommands.registerCommand("Shoot Amp", new ShootAmp(intakeIndexer, shooter));
    NamedCommands.registerCommand("Intake From Source", new IntakeFromSource(intakeIndexer, shooter));
    NamedCommands.registerCommand("Intake Note", new IntakeNote(intakeIndexer, intakeIndexer.hasNote()));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    driveController.a().onTrue(new InstantCommand(() -> {
      drivetrain.zeroGyro();
    }, drivetrain));

    // Left red button - Intake
    manipulatorController.leftBumper().toggleOnTrue(new IntakeNote(intakeIndexer, intakeIndexer.hasNote()));

    // Right red button - Source
    manipulatorController.povDown().onTrue(new IntakeFromSource(intakeIndexer, shooter));
    
    // Left yellow button - A (Amp) Shoot
    manipulatorController.b().onTrue(new ShootAmp(intakeIndexer, shooter));

    // Right yellow button - Shoot
    manipulatorController.rightBumper().onTrue(new ShootSpeaker(intakeIndexer, shooter));

    // Top blue button - Climb
    manipulatorController.y().onTrue(
      new InstantCommand(() -> {
        pneumatics.toggleClimber();
      }, pneumatics)
    );
    
    // Bottom blue button - Panel
    manipulatorController.x().onTrue(
      new InstantCommand(() -> {
        pneumatics.toggleAmpGuide();    
      }, pneumatics)
    );

    // Black button - spit
    manipulatorController.leftTrigger().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> intakeIndexer.startSpittingNote()),
      new WaitCommand(1),
      new InstantCommand(() -> intakeIndexer.stop())
    ));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public FrontLimelight getFrontLimelight() {
    return frontLimelight;
  }

  public BackLimelight getBackLimelight() {
    return backLimelight;
  }

  public IntakeIndexer getIntakeindexer() {
      return intakeIndexer;
  }

  public Swerve getDrivetrain() {
      return drivetrain;
  }
}
