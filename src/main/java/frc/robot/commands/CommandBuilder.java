// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.components.AdvanceToTarget;
import frc.robot.commands.components.AlignNote;
import frc.robot.commands.components.IntakeNote;
import frc.robot.commands.components.PrepareToShoot;
import frc.robot.subsystems.BackLimelight;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class CommandBuilder {


    public static Command IntakeNote(IntakeIndexer intakeIndexer) {
        return new IntakeNote(intakeIndexer)
            .until(intakeIndexer::getLimitSwitch);
    }

    public static Command ShootSpeaker(IntakeIndexer intakeIndexer, Shooter shooter) {
        return new PrepareToShoot(shooter, Constants.Launcher.speakerSpeed)
            .alongWith(
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
                new WaitCommand(0.5).andThen(() -> {intakeIndexer.setSpeed(0.5);}, intakeIndexer),
                new InstantCommand(() -> {pneumatics.extendAmpGuide();}, pneumatics)
            )
            .raceWith(new WaitCommand(2))
            .andThen(() -> {
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
            .andThen(() -> {intakeIndexer.setSpeed(-0.35); System.out.println(intakeIndexer.getSpeed());}, intakeIndexer)
            .andThen(new WaitCommand(1.5))
            .andThen(() -> {intakeIndexer.stop(); System.out.println("edge");}, intakeIndexer);
    } 

    public static Command AdvanceToTarget(Swerve swervedrive, Field2d playingField, Pose2d targetPose) {
        return new AdvanceToTarget(swervedrive, playingField, targetPose);
    }

    public static Command AlignNote(Swerve drivetrain, Field2d playingField, BackLimelight backLimelightInstance) {
        return new AlignNote(drivetrain, playingField, backLimelightInstance);
    }

    public static Command AutomaticNoteIntake(IntakeIndexer intakeIndexer, Swerve drivetrain, Field2d playingField, BackLimelight backLimelightInstance) {
        return Commands.parallel(
            new AlignNote(drivetrain, playingField, backLimelightInstance),
            new IntakeNote(intakeIndexer)
        );
    }
}
