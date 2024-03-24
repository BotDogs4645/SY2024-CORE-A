// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.DistanceEstimation;
import frc.robot.commands.CommandBuilder;
import frc.robot.commands.components.TeleopSwerve;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.BackLimelight;
import frc.robot.subsystems.FrontLimelight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 *  commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private final Field2d playingField = new Field2d();

  private final CommandXboxController driveController = new CommandXboxController(Constants.kDriverControllerPort);
  private final CommandXboxController manipulatorController = new CommandXboxController(Constants.kManipulatorControllerPort);

  private final DistanceEstimation distanceEstimation = new DistanceEstimation();

  private final Swerve drivetrain = new Swerve(playingField);
  private final FrontLimelight frontLimelight = new FrontLimelight();
  private final BackLimelight backLimelight = new BackLimelight(distanceEstimation);
  private final Pneumatics pneumatics = new Pneumatics();
  private final IntakeIndexer intakeIndexer = new IntakeIndexer();
  private final Shooter shooter = new Shooter();

  public RobotContainer() {

    NamedCommands.registerCommand("Shoot Speaker", CommandBuilder.ShootSpeaker(intakeIndexer, shooter));
    NamedCommands.registerCommand("Shoot Amp", CommandBuilder.ShootAmp(intakeIndexer, shooter, pneumatics));
    NamedCommands.registerCommand("Intake Note", CommandBuilder.IntakeNote(intakeIndexer));
    NamedCommands.registerCommand("Automatically Intake Note", CommandBuilder.AutomaticNoteIntake(intakeIndexer, drivetrain, playingField, backLimelight));

    

    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            () -> -driveController.getLeftY(), // Translation
            () -> -driveController.getLeftX(), // Strafe
            () -> -driveController.getRightX(), // Rotation
            () -> driveController.rightTrigger().getAsBoolean() // Field-oriented driving (yes or no)
        )
    );

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // SmartDashboard.putData("Auto Chooser", );

    configureBindings();
  }

  private void configureBindings() {
    driveController.a().onTrue(new InstantCommand(() -> {
      drivetrain.zeroGyro();
    }, drivetrain));

    // Left red button - Intake
    // manipulatorController.leftBumper().toggleOnTrue(CommandBuilder.IntakeNote(intakeIndexer));
    manipulatorController.leftBumper().toggleOnTrue(CommandBuilder.AutomaticNoteIntake(intakeIndexer, drivetrain, playingField, backLimelight));
    
    // Right red button - Source
    // manipulatorController.povDown().onTrue(new IntakeFromSource(intakeIndexer, shooter));
    manipulatorController.povDown().onTrue(CommandBuilder.IntakeFromSource(intakeIndexer, shooter));
    
    // Left yellow button - A (Amp) Shoot
    manipulatorController.b().onTrue(CommandBuilder.ShootAmp(intakeIndexer, shooter, pneumatics));

    // Right yellow button - Shoot
    manipulatorController.rightBumper().onTrue(CommandBuilder.ShootSpeaker(intakeIndexer, shooter));

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
