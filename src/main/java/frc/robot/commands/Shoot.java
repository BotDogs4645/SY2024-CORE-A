// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  Launcher launcher;
  IntakeIndexer intakeIndexer;
  Limelight limelight;
  public Shoot(Launcher launcher, IntakeIndexer intakeIndexer, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcher = launcher;
    this.intakeIndexer = intakeIndexer;
    this.limelight = limelight;
    addRequirements(launcher, intakeIndexer, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcher.aimLauncher();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (/*need node id to determine that we are at an amp*/){
      vroom(Constants.Launcher.ampSpeed);
    }
    else if (/*need node id to determine that we are at a speaker*/){
      vroom(Constants.Launcher.speakerSpeed);
    }
    else if (/*need node id to determine that we are at a trap*/){
      vroom(Constants.Launcher.trapSpeed);
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.stopLauncher();
    intakeIndexer.stopIntake();
    intakeIndexer.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public void vroom(double speed){
    intakeIndexer.runIntake(speed);
    intakeIndexer.runFeeder(speed);
    launcher.startLauncher(speed);
  }
}
