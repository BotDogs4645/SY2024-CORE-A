// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.AprilTag;
import frc.lib.util.NodeStorage;
import frc.robot.subsystems.Launcher;
import frc.robot.commands.AdvanceToTarget;
import frc.robot.commands.IntakeIndexerCommand;
import frc.robot.commands.NodalTaskExecution;
import frc.robot.commands.Shoot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;
import java.util.Optional;

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

  public static final CommandXboxController driveController = new CommandXboxController(Constants.kDriverControllerPort);

  private final Field2d playingField = new Field2d();

  private final Swerve drivetrain = new Swerve(playingField);
  private final Pneumatics pneumatics = new Pneumatics();

  private final Limelight limelightInstance = new Limelight();
  private final AprilTag aprilTagInstance = new AprilTag(limelightInstance);

  private final IntakeIndexer intakeIndexerInstance = new IntakeIndexer();

  private final Launcher launcherInstance = new Launcher();

  private final NodeStorage nodeStorageInstance = new NodeStorage(drivetrain, playingField, launcherInstance, intakeIndexerInstance);

  Optional<IntakeIndexerCommand> intakeIndexerCommandInstance = Optional.empty();
  Optional<NodalTaskExecution> nodalTaskExecutionInstance = Optional.empty();
  Optional<Shoot> shootInstance = Optional.empty();
  
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            () -> -driveController.getLeftY(), // translation
            () -> -driveController.getLeftX(), // strafe
            () -> -driveController.getRightX(), // rotation
            () -> driveController.leftBumper().getAsBoolean() // field oriented, yes or no
        ).finallyDo(() -> {
          intakeIndexerInstance.stopIntake();
          intakeIndexerInstance.stopFeeder();
        }));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putData("Field", playingField);

    configureBindings();
  }

  private void configureBindings() {
    driveController.a().onTrue(new InstantCommand(() -> {
      drivetrain.zeroGyro();
    }, drivetrain));

    driveController.leftBumper().onTrue(Commands.run(
      () -> {
        if (intakeIndexerCommandInstance.isEmpty() || !intakeIndexerCommandInstance.get().isScheduled()) {
          intakeIndexerCommandInstance = Optional.of(new IntakeIndexerCommand(intakeIndexerInstance));
        } else if (intakeIndexerCommandInstance.isPresent() && intakeIndexerCommandInstance.get().isScheduled()) {
          intakeIndexerCommandInstance.get().cancel();
          intakeIndexerCommandInstance = Optional.empty();
        }
    }));
    driveController.rightBumper().onTrue(Commands.run(
      () -> {
        if (!shootInstance.isEmpty() || !shootInstance.get().isScheduled()){
          shootInstance = Optional.of(new Shoot(launcherInstance, intakeIndexerInstance, 0));
        } else if(shootInstance.isPresent() && shootInstance.get().isScheduled()){
          shootInstance.get().cancel();
          shootInstance = Optional.empty();
        }
      }
      ));

    // driveController.leftTrigger().onTrue(new SequentialCommandGroup(
    //   new InstantCommand(() -> intakeIndexerInstance.unloadIntakeIndexer()),
    //   new WaitCommand(1.5),
    //   new InstantCommand(() -> intakeIndexerInstance.haltIntakeIndexer())
    // ));

    driveController.rightTrigger().onTrue(Commands.run(
      () -> {
        if (nodalTaskExecutionInstance.isEmpty() || !nodalTaskExecutionInstance.get().isScheduled()) {
          nodalTaskExecutionInstance = Optional.of(new NodalTaskExecution(aprilTagInstance, playingField, nodeStorageInstance));
        } else if (intakeIndexerCommandInstance.isPresent() && intakeIndexerCommandInstance.get().isScheduled()) {
          nodalTaskExecutionInstance.get().cancel();
          nodalTaskExecutionInstance = Optional.empty();
        }
      }
    ));

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
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public AprilTag getAprilTag() {
    return aprilTagInstance;
  }

  public Swerve getDrivetrain() {
      return drivetrain;
  }

  public Field2d getField() {
    return playingField;
  }
}
