// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeIndexer;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */

  IntakeIndexer intakeIndexer;
  boolean hasNoteAlready;

  public IntakeNote(IntakeIndexer intakeIndexer, boolean hasNoteAlready) {
    this.hasNoteAlready = hasNoteAlready;
    this.intakeIndexer = intakeIndexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(intakeIndexer.getTripTime()) > 1) {
      intakeIndexer.setSpeed(Constants.Intake.intakeSpeed);
      super.cancel();
    } 

    if(intakeIndexer.getLimitSwitch() && !hasNoteAlready) {
      intakeIndexer.setSpeed(0);
      intakeIndexer.setHasNote(true);
      super.cancel();
    } else {
      intakeIndexer.setSpeed(Constants.Intake.intakeSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      intakeIndexer.setSpeed(0);
      intakeIndexer.resetSwitch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
