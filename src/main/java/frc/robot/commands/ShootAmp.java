// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends Command {
  /** Creates a new ShootAmp. */
  private Shooter shooter;
  private IntakeIndexer intakeIndexer;

  public ShootAmp(IntakeIndexer intakeIndexer, Shooter shooter) {
    this.shooter = shooter;
    this.intakeIndexer = intakeIndexer;

    addRequirements(shooter, intakeIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Commands.parallel(
      new InstantCommand(() -> {
        shooter.toggleShooterAmp();
        shooter.setShooterAngle(0.1);
      }, shooter),
      new WaitCommand(0.5).andThen(
        new InstantCommand(() -> {
          intakeIndexer.toggle();
        }, intakeIndexer)
      )
    ).andThen(
      new WaitCommand(1).andThen(() -> {
        intakeIndexer.toggle(); 
        shooter.toggleShooterAmp();
      }, intakeIndexer, shooter)
    ).schedule();
    super.cancel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
