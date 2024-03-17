// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.components;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeIndexer;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */

  private IntakeIndexer intakeIndexer;

  public IntakeNote(IntakeIndexer intakeIndexer) {
    this.intakeIndexer = intakeIndexer;

    addRequirements(intakeIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeIndexer.setSpeed(Constants.Intake.intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeIndexer.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
