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

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.lib.util.AprilTag;
import frc.lib.util.ObstacleDetection;

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
        Optional<Transform3d> currentPosition = aprilTagInstance.determinePosition();
        if (currentPosition.isEmpty()) {
            return;
        }

        if (ObstacleDetection.continueAlongPath(currentPosition.get().getTranslation(), (targetPosition.getY() - currentPosition.get().getY()) / (targetPosition.getX() - currentPosition.get().getX()))) {
            this.targetPosition = Optional.of(targetPosition);
        } else {
            this.targetPosition = Optional.empty();
        }
    }

    @Override
    public void execute() {
        if (!isFinished() && targetPosition.isPresent() && aprilTagInstance.validTargetInput(Optional.of(targetPosition.get().getTranslation()))) {
            double xAngle = aprilTagInstance.determineTargetRotationalOffset(Optional.of(targetPosition.get().getTranslation())).get()[0];

            double translationVal;
            double strafeVal;
            double rotationVal;


            if (xAngle < Constants.AdvanceToTarget.minAngle) {
                translationVal = 0;
                strafeVal = 0;
                rotationVal = Constants.AdvanceToTarget.swerveRotationalValue;
            } else if (xAngle > Constants.AdvanceToTarget.maxAngle) {
                translationVal = 0;
                strafeVal = 0;
                rotationVal = -Constants.AdvanceToTarget.swerveRotationalValue;
            } else {
                translationVal = Constants.AdvanceToTarget.swerveTranslationValue;
                strafeVal = 0;
                rotationVal = 0;
            }

            translationVal =
                translationLimiter.calculate(
                    MathUtil.applyDeadband(translationVal, Constants.Swerve.stickDeadband));
            strafeVal =
                strafeLimiter.calculate(
                    MathUtil.applyDeadband(strafeVal, Constants.Swerve.stickDeadband));
            rotationVal =
                rotationLimiter.calculate(
                    MathUtil.applyDeadband(rotationVal, Constants.Swerve.stickDeadband));

            swerveDrive.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                !robotActivated,
                true);
        } else {
            targetPosition = Optional.empty();
        }
    }

    // @Override
    // public void end(boolean interrupted) {

    //     if (aprilTagInstance.targetPos() == null) {
    //         CommandScheduler.getInstance().schedule(new AdvanceToTarget(swerveDrive, aprilTagInstance));
    //     }
    // }

    @Override
    public boolean isFinished() {
        var targetPos = aprilTagInstance.targetPos();
        
        return targetPos != null && aprilTagInstance.getDirectDistance(targetPosition).get() < Constants.AdvanceToTarget.maximumLaunchDistance;
    }
}