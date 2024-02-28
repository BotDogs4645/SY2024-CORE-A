// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import javax.swing.plaf.InsetsUIResource;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Launcher;

public class Shoot extends Command {
  private Launcher launcher;
  private IntakeIndexer intakeIndexer;
  private DoubleSupplier rightTrigger;

  /** Creates a new Shoot. */
  public Shoot(Launcher launcher, IntakeIndexer intakeIndexer, DoubleSupplier rightTrigger) {
    this.launcher = launcher;
    this.intakeIndexer = intakeIndexer;
    this.rightTrigger = rightTrigger;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher, intakeIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new SequentialCommandGroup(
      new InstantCommand(() -> launcher.startLauncher(0.2)),
      new InstantCommand(() -> launcher.aimLauncher()),
      new WaitCommand(1),
      new InstantCommand(() -> intakeIndexer.run()),
      new WaitCommand(3),
      new InstantCommand(() -> intakeIndexer.stop()),
      new InstantCommand(() -> intakeIndexer.setHasNote(false)),
      new InstantCommand(() -> launcher.stopLauncher())
    ).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      intakeIndexer.stop();
      launcher.stopLauncher();
      intakeIndexer.setHasNote(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
