// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.components;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterToCrossFieldPos extends Command {
  /** Creates a new ShooterToCrossFieldPos. */
  
  private Shooter shooter;
  private double shooterSpeed;


  public ShooterToCrossFieldPos(Shooter shooter, double shooterSpeed) {
    this.shooter = shooter;
    this.shooterSpeed = shooterSpeed;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeed(shooterSpeed);
    shooter.setShooterAngle(0.15);
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
    return true;
  }
}
