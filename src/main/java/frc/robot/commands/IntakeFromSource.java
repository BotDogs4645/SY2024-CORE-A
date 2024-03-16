// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromSource extends SequentialCommandGroup {

  /** Creates a new IntakeFromSource. */
  public IntakeFromSource(IntakeIndexer intakeIndexer, Shooter shooter) {

    addCommands(
      new InstantCommand(() -> {
        shooter.intakeFromSource();
        intakeIndexer.setSpeed(-0.125);
      }, intakeIndexer, shooter),
      new WaitCommand(0.5),
      new InstantCommand(() -> {
          shooter.setShooterSpeed(0);
          intakeIndexer.setSpeed(0);
      }, shooter, intakeIndexer)
    );
  }
}
