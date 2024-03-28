// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.components.StartIntakeNote;
import frc.robot.commands.components.PrepareToShoot;
import frc.robot.commands.components.ShooterToCrossFieldPos;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class CommandBuilder {


    public static Command IntakeNote(IntakeIndexer intakeIndexer) {
        return new StartIntakeNote(intakeIndexer)
            .until(intakeIndexer::getPhotogate)
            .andThen(() -> {intakeIndexer.setSpeed(0.15);}, intakeIndexer)
            .andThen(new WaitCommand(0.15))
            .andThen(() -> {intakeIndexer.stop();}, intakeIndexer);
    }

    public static Command Shoot(IntakeIndexer intakeIndexer, Shooter shooter) {
        Command cmd;

        if(RobotContainer.manipulatorController.a().getAsBoolean()) {
            cmd = new ShooterToCrossFieldPos(shooter, 0.75);
        } else {
            cmd = new PrepareToShoot(shooter, Constants.Launcher.speakerSpeed);
        }

        return cmd.alongWith(
                new WaitCommand(0.5).andThen(() -> {intakeIndexer.toggle();}, intakeIndexer)
            )
            .andThen(new WaitCommand(1))
            .andThen(() -> {
                intakeIndexer.toggle();
                shooter.stop();
            }, intakeIndexer, shooter);
    
    }

    public static Command ShootAmp(IntakeIndexer intakeIndexer, Shooter shooter, Pneumatics pneumatics) {
        return new PrepareToShoot(shooter, Constants.Launcher.ampSpeed)
            .alongWith(
                new WaitCommand(0.5).andThen(() -> { System.out.println("intake"); intakeIndexer.setSpeed(0.5);}, intakeIndexer),
                new InstantCommand(() -> { System.out.println("piston"); pneumatics.extendAmpGuide();}, pneumatics)
            )
            .raceWith(new WaitCommand(2))
            .andThen(() -> {
                System.out.println("Stopped");
                intakeIndexer.stop();
                shooter.stop();
            });
    }

    public static Command IntakeFromSource(IntakeIndexer intakeIndexer, Shooter shooter) {
        return Commands.run(() -> {
            shooter.intakeFromSource();
            intakeIndexer.intakeFromSource();
        }, shooter, intakeIndexer)
            .until(intakeIndexer::getPhotogate)
            .andThen(() -> {intakeIndexer.setSpeed(-0.35);}, intakeIndexer)
            .andThen(new WaitCommand(1.5))
            .andThen(() -> {intakeIndexer.stop(); shooter.stop();}, intakeIndexer,shooter);
    } 

    public static Command ShootAcrossField(IntakeIndexer intakeIndexer, Shooter shooter) {
        return new InstantCommand();
    }
}
