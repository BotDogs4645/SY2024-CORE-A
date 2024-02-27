// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Limelight;

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
    if (intakeIndexer.hasNote()){
      intakeIndexer.runIntake(0.5);
    }
    else{
      intakeIndexer.stopIntake();
    }
      intakeIndexer.runFeeder(0.5);
      launcher.startLauncher(0.5);
      //put delay here (without causing a scheduling error)
      launcher.stopLauncher();
      launcher.aimLauncher();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.startLauncher(limelight.getLaunchVelocity().getAsDouble());
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
}
