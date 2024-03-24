// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.components;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import java.util.List;

public class AdvanceToTarget extends Command {

    private Swerve swerveDrive;
    private Field2d playingField;
    private Optional<Pose2d> targetPose;
    public Optional<SwerveControllerCommand> swerveControllerCommand = Optional.empty();

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    public AdvanceToTarget(Swerve swerveDrive, Field2d playingField, Pose2d targetPose) {
        this.swerveDrive = swerveDrive;
        this.playingField = playingField;
        
        this.targetPose = Optional.of(targetPose);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        Optional<Pose2d> currentPosition = Optional.of(playingField.getRobotPose());
        if (currentPosition.isEmpty() || targetPose.isEmpty() || (Math.hypot(targetPose.get().getX(), targetPose.get().getY()) < Constants.AdvanceToTarget.minimumProjectedDistance)) {
            super.cancel();
            return;
        }

        List<Translation2d> trajectoryWaypoints = List.of();
        

        // for (Translation2d currentWaypoint : trajectoryWaypoints) {
        //     if (!ObstacleDetection.continueAlongPath(currentPosition.get().getTranslation(), (currentWaypoint.getY() - currentPosition.get().getY()) / (currentWaypoint.getX() - currentPosition.get().getX()))) {
        //         return;
        //     }
        // }


        TrajectoryConfig trajectoryConfig =
            new TrajectoryConfig(
                    Constants.Swerve.maxSpeed / Constants.AdvanceToTarget.speedDilutionFactor,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared / Constants.AdvanceToTarget.accelerationDilutionFactor)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.Swerve.swerveKinematics);
    
        Trajectory projectedTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                trajectoryWaypoints,
                targetPose.get(),
                trajectoryConfig);
    
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        swerveControllerCommand = Optional.of(
            new SwerveControllerCommand(
                projectedTrajectory,
                swerveDrive::getPose, // Functional interface to feed supplier
                Constants.Swerve.swerveKinematics,
    
                // Position controllers
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                states -> swerveDrive.setModuleStates(states, false),
                swerveDrive));

        swerveControllerCommand.get().finallyDo(() -> swerveDrive.drive(
            new Translation2d(0, 0),
            0,
            false,
            true));


        swerveControllerCommand.get().schedule();
    }

    @Override
    public void end(boolean interrupted) {
        swerveControllerCommand.get().cancel();
        
        swerveDrive.drive(
            new Translation2d(0, 0),
            0,
            false,
            true);
    }

    @Override
    public boolean isFinished() {
        if (swerveControllerCommand.isPresent()) {
            return swerveControllerCommand.get().isFinished();
        } else {
            return false;
        }
    }
}