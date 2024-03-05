// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.Optional;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.lib.util.ObstacleDetection;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import java.util.List;

public class AdvanceToTarget extends Command {

    private Swerve swerveDrive;
    private Field2d playingField;
    private boolean fieldOrientated;
    private Optional<Transform2d> targetPosition;
    public Optional<SequentialCommandGroup> currentTargetCommand = Optional.empty();

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    public AdvanceToTarget(Swerve swerveDrive, Field2d playingField, boolean fieldOrientated, Transform2d targetPosition) {
        this.swerveDrive = swerveDrive;
        this.playingField = playingField;
        this.fieldOrientated = fieldOrientated;
        
        this.targetPosition = Optional.of(targetPosition);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        Optional<Pose2d> currentPosition = Optional.of(playingField.getRobotPose());
        if (currentPosition.isEmpty() || targetPosition.isEmpty()) {
            return;
        }

        // List<Translation2d> trajectoryWaypoints = List.of(new Translation2d(1, 1), new Translation2d(2, -1), targetPosition.get().getTranslation().toTranslation2d());
        List<Translation2d> trajectoryWaypoints = List.of(targetPosition.get().getTranslation());

        for (Translation2d currentWaypoint : trajectoryWaypoints) {
            if (!ObstacleDetection.continueAlongPath(currentPosition.get().getTranslation(), (currentWaypoint.getY() - currentPosition.get().getY()) / (currentWaypoint.getX() - currentPosition.get().getX()))) {
                return;
            }
        }


        TrajectoryConfig trajectoryConfig =
            new TrajectoryConfig(
                    Constants.Swerve.maxSpeed,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.Swerve.swerveKinematics);
    
        Trajectory projectedTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(currentPosition.get().getTranslation(), targetPosition.get().getRotation()),
                trajectoryWaypoints,
                new Pose2d(targetPosition.get().getTranslation(), targetPosition.get().getRotation()),
                trajectoryConfig);
    
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                projectedTrajectory,
                swerveDrive::getPose, // Functional interface to feed supplier
                Constants.Swerve.swerveKinematics,
    
                // Position controllers
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerveDrive::setModuleStates,
                swerveDrive);
    
        // Run path following command, then stop at the end.
        currentTargetCommand = Optional.of(swerveControllerCommand.andThen(() -> swerveDrive.drive(
            new Translation2d(0, 0),
            0,
            !fieldOrientated,
            true)).andThen(() -> this.targetPosition = Optional.empty()));

        currentTargetCommand.get().schedule();
    }

    @Override
    public void end(boolean interrupted) {
        currentTargetCommand.get().cancel();
        
        swerveDrive.drive(
            new Translation2d(0, 0),
            0,
            !fieldOrientated,
            true);
    }

    @Override
    public boolean isFinished() {
        return targetPosition.isEmpty() || currentTargetCommand.isEmpty() || currentTargetCommand.get().isFinished();
    }
}