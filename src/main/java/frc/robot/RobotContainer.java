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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.AprilTag;
import frc.robot.Constants;
import frc.robot.commands.AdvanceToTarget;
import frc.robot.commands.IntakeIndexerCommand;
import frc.robot.commands.NodalTaskExecution;
import frc.robot.commands.Shoot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Launcher;

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
  private final Swerve swerveInstance = new Swerve(playingField);
  private final Pneumatics m_pneumaticsSubsystem = new Pneumatics();

  private final Limelight limelightInstance = new Limelight();
  private final AprilTag aprilTagInstance = new AprilTag(limelightInstance);
  private final AdvanceToTarget advanceToTargetInstance = new AdvanceToTarget(drivetrain, aprilTagInstance, true);
  private final NodalTaskExecution nodalTaskExecutionInstance = new NodalTaskExecution(aprilTagInstance, advanceToTargetInstance, playingField);

  private final IntakeIndexer intakeIndexerInstance = new IntakeIndexer();
  private final IntakeIndexerCommand intakeIndexerCommand = new IntakeIndexerCommand(intakeIndexerInstance);
  private final Launcher launcherInstance = new Launcher();
  private final Shoot shoot = new Shoot(swerveInstance, launcherInstance, intakeIndexerInstance, aprilTagInstance, nodalTaskExecutionInstance);
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

          intakeIndexerCommand.cancel();
          nodalTaskExecutionInstance.cancel();
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

    driveController.leftTrigger().onTrue(Commands.run(
      () -> {
        if (!intakeIndexerCommand.isScheduled()) {
          intakeIndexerCommand.schedule();
        } else {
          intakeIndexerCommand.cancel();
        }
    }));

    driveController.rightTrigger().onTrue(Commands.run(
      () -> {
        if (!nodalTaskExecutionInstance.isScheduled()) {
          nodalTaskExecutionInstance.schedule();
        } else {
          nodalTaskExecutionInstance.cancel();
        }
      }
    ));

    driveController.rightBumper().onTrue(Commands.run(
      () -> {
        if (!shoot.isScheduled()){
          shoot.schedule();
        } else {
          shoot.cancel();
        }
      }
      ));

    // driveController.y().onTrue(new DriveToTag(
    //   drivetrain,
    //   limelight.getTargetPose()
    // ));
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

  public AdvanceToTarget getAdvanceToTarget() {
      return advanceToTargetInstance;
  }
}
