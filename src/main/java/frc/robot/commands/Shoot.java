// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import java.util.Optional;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  Swerve swerveInstance;
  Launcher launcherInstance;
  IntakeIndexer intakeIndexerInstance;
  int currentNodeID;

  Optional<Long> initalizationTime;
  
  public Shoot(Launcher launcher, IntakeIndexer intakeIndexer, int currentNodeID) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcherInstance = launcher;
    this.intakeIndexerInstance = intakeIndexer;
    this.currentNodeID = currentNodeID;

    addRequirements(launcherInstance, intakeIndexerInstance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initalizationTime = Optional.of(System.currentTimeMillis());

    if (currentNodeID == Constants.Launcher.ampNodeID){
      vroom(Constants.Launcher.ampSpeed);
      launcherInstance.aimLauncher(Constants.Launcher.ampAngle);
    }
    else if (currentNodeID == Constants.Launcher.speakerNodeID){
      vroom(Constants.Launcher.speakerSpeed);
      launcherInstance.aimLauncher(Constants.Launcher.speakerAngle);
    }
    else if (currentNodeID == Constants.Launcher.trapNodeID){
      vroom(Constants.Launcher.trapSpeed);
      launcherInstance.aimLauncher(Constants.Launcher.trapAngle);
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherInstance.stopLauncher();
    intakeIndexerInstance.stopIntake();
    intakeIndexerInstance.stopFeeder();

    initalizationTime = Optional.empty();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return initalizationTime.isEmpty() || System.currentTimeMillis() - initalizationTime.get() > Constants.Launcher.executionDurationMillis;
  }

  public void vroom(double speed){
    intakeIndexerInstance.runIntake(speed);
    intakeIndexerInstance.runFeeder(speed);
    launcherInstance.startLauncher(speed);
  }
}