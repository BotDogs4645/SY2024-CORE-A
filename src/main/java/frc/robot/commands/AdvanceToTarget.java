// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.Function;

import com.sun.org.apache.bcel.internal.Const;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.lib.util.AprilTag;
import frc.lib.util.ObstacleDetection;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import java.util.List;
import java.lang.Runnable;
public class AdvanceToTarget extends Command {

    private Swerve swerveDrive;
    private AprilTag aprilTagInstance;
    private boolean robotActivated;
    private Optional<Transform3d> targetPosition;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    public AdvanceToTarget(Swerve swerveDrive, AprilTag aprilTagInstance, boolean robotActivated) {
        this.swerveDrive = swerveDrive;
        this.aprilTagInstance = aprilTagInstance;
        this.robotActivated = robotActivated;
        addRequirements(swerveDrive);
    }

    public void specifyTarget(Transform3d targetPosition) {
        this.targetPosition = Optional.of(targetPosition);
    }

    @Override
    public void initialize() {
        Optional<Transform3d> currentPosition = aprilTagInstance.determinePosition();
        if (currentPosition.isEmpty() || targetPosition.isEmpty()) {
            return;
        }

        List<Translation2d> trajectoryWaypoints = List.of(new Translation2d(1, 1), new Translation2d(2, -1), targetPosition.get().getTranslation().toTranslation2d());

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
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through the two interior waypoints defined below, constructing an 's' curve path
                // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                List.of(),
                new Pose2d(targetPosition.getTranslation().toTranslation2d(), targetPosition.getRotation().toRotation2d()),
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
    
        // Reset odometry to the starting pose of the trajectory.
        // swerveDrive.resetOdometry(exampleTrajectory.getInitialPose()); // As of 2/26/2024, I am unsure as to why this line of code was included in the example resources from which this format is derived. - Carver
    
        // Run path following command, then stop at the end.
        swerveControllerCommand.andThen(() -> swerveDrive.drive(
            new Translation2d(0, 0),
            0,
            !robotActivated,
            true)).andThen(() -> this.targetPosition = Optional.empty()).schedule();
    }

    // @Override
    // public void execute() {
    //     if (targetPosition.isPresent() && aprilTagInstance.validTargetInput(Optional.of(targetPosition.get().getTranslation()))) {
    //         double xAngle = aprilTagInstance.determineTargetRotationalOffset(Optional.of(targetPosition.get().getTranslation())).get()[0];

    //         double translationVal;
    //         double strafeVal;
    //         double rotationVal;


    //         if (xAngle < Constants.AdvanceToTarget.minAngle) {
    //             translationVal = 0;
    //             strafeVal = 0;
    //             rotationVal = Constants.AdvanceToTarget.swerveRotationalValue;
    //         } else if (xAngle > Constants.AdvanceToTarget.maxAngle) {
    //             translationVal = 0;
    //             strafeVal = 0;
    //             rotationVal = -Constants.AdvanceToTarget.swerveRotationalValue;
    //         } else {
    //             translationVal = Constants.AdvanceToTarget.swerveTranslationValue;
    //             strafeVal = 0;
    //             rotationVal = 0;
    //         }

    //         translationVal =
    //             translationLimiter.calculate(
    //                 MathUtil.applyDeadband(translationVal, Constants.Swerve.stickDeadband));
    //         strafeVal =
    //             strafeLimiter.calculate(
    //                 MathUtil.applyDeadband(strafeVal, Constants.Swerve.stickDeadband));
    //         rotationVal =
    //             rotationLimiter.calculate(
    //                 MathUtil.applyDeadband(rotationVal, Constants.Swerve.stickDeadband));

    //         swerveDrive.drive(
    //             new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
    //             rotationVal * Constants.Swerve.maxAngularVelocity,
    //             !robotActivated,
    //             true);
    //     } else {
    //         swerveDrive.drive(
    //             new Translation2d(0, 0),
    //             0,
    //             !robotActivated,
    //             true);

    //         targetPosition = Optional.empty();
    //     }
    // }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(
            new Translation2d(0, 0),
            0,
            !robotActivated,
            true);
    }

    @Override
    public boolean isFinished() {
        Optional<Transform3d> targetPos = aprilTagInstance.targetPos();
        
        return !targetPos.isPresent() && !targetPosition.isPresent();
    }
}