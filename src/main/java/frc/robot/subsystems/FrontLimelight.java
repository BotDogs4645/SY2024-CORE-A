package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/**
 * The Limelight subsystem.
 * 
 * Limelight documentation: https://limelightvision.io/
 * 
 * We use the Limelight for location estimation with AprilTags.
 */
public class FrontLimelight extends SubsystemBase {

    private OptionalDouble distanceToTarget = OptionalDouble.empty();
    private OptionalDouble ty = OptionalDouble.empty();
    private OptionalDouble tx = OptionalDouble.empty();


    public boolean hasTarget() {
        return (LimelightHelpers.getTA(Vision.FrontLimelight.Name) > 0.1) ? true : false;
    }

    public OptionalDouble getLateralAngleToTarget() {
        return tx;
    }

    public OptionalDouble getVerticalAngleToTarget() {
        return ty;
    }

    public OptionalInt getTagID() {
        try {
            return OptionalInt.of((int) LimelightHelpers.getFiducialID(Vision.FrontLimelight.Name));
        } catch (Exception e) {
            return OptionalInt.empty();
        }
    }

    public OptionalDouble getDistanceToTarget() {
        if (!ty.isPresent() && !hasTarget()) {
            return OptionalDouble.empty();
        }
        double d = (
            (Constants.Vision.FrontLimelight.Up - Constants.Limelight.APRILTAGS.get(
                (int) LimelightHelpers.getFiducialID(Vision.FrontLimelight.Name)).getZ()
            ) / Math.tan(Constants.Vision.FrontLimelight.Pitch + ty.getAsDouble())
        ); 

        return OptionalDouble.of(d);
    }

  
    public Optional<Pose3d> getTargetPoseRobotRelative() {
        try {
            return Optional.of(LimelightHelpers.getTargetPose3d_RobotSpace(Vision.FrontLimelight.Name));             
        } catch (Exception e) {
            return Optional.empty();
        }
    }

    public Optional<Pose2d> getRobotPosition_FieldSpace() {
        if (!hasTarget()) {
            return Optional.empty();
        }

        return Optional.of(LimelightHelpers.getBotPose2d_wpiBlue(Vision.FrontLimelight.Name))
            .filter(pose -> pose.getX() != 0 && pose.getY() != 0);
    }

    public OptionalDouble getVerticalVelocity() {
        if (hasTarget()) {
            double heightDifference = Constants.Limelight.APRILTAGS.get(getTagID().getAsInt()).getZ()
                    - Constants.Launcher.launcherHeight;
            return OptionalDouble.of(Math.sqrt(
                    2 * Constants.Launcher.gravityAcceleration * (Constants.Limelight.APRILTAGS.get(getTagID().getAsInt()).getZ())
                            - Constants.Launcher.launcherHeight));
        } else {
            return OptionalDouble.empty();
        }
    }

    public OptionalDouble getHorizontalVelocity() {
        if (!hasTarget()) {
            return OptionalDouble.empty();
        }

        double timeToTravel = (-1 * getVerticalVelocity().getAsDouble() + Math.sqrt(
                Math.pow(getVerticalVelocity().getAsDouble(), 2)
                        + 2 * Constants.Launcher.gravityAcceleration
                                * (Constants.Limelight.APRILTAGS.get(getTagID().getAsInt()).getZ())
                        - Constants.Launcher.launcherHeight))
                / (Constants.Launcher.gravityAcceleration);
        return OptionalDouble.of(getDistanceToTarget().getAsDouble() / timeToTravel);
    }

    public OptionalDouble getLaunchVelocity() {
        if (hasTarget()) {
            return OptionalDouble
                    .of(toRPM(Math.hypot(getHorizontalVelocity().getAsDouble(), getVerticalVelocity().getAsDouble())));
        }
        return OptionalDouble.empty();
    }

    public OptionalDouble getLaunchAngle() {
        if (hasTarget()) {
            return OptionalDouble.of(Math
                    .toDegrees(Math.atan(getVerticalVelocity().getAsDouble() / getHorizontalVelocity().getAsDouble())));
        }
        return OptionalDouble.empty();
    }

    /**
     * Converts a velocity to RPM (Rotations Per Minute).
     * 
     * @param velocity the velocity to convert
     * @return the converted RPM value
     */
    public double toRPM(double velocity) {
        return velocity * 60 / (2 * Math.PI * Constants.Launcher.launcherWheelRadius);
    }

    @Override
    public void periodic() {
        if(hasTarget()) {
            tx = OptionalDouble.of(LimelightHelpers.getTX(Vision.FrontLimelight.Name));
            ty = OptionalDouble.of(LimelightHelpers.getTY(Vision.FrontLimelight.Name));
            distanceToTarget = getDistanceToTarget();
        } else {
            tx = OptionalDouble.empty();
            ty = OptionalDouble.empty();
            distanceToTarget = OptionalDouble.empty();
        }
        SmartDashboard.putNumber("LL Distance", getDistanceToTarget().orElse(-1));
        try {
            SmartDashboard.putNumber("LL From Pose X", getTargetPoseRobotRelative().get().getX());
            SmartDashboard.putNumber("LL From Pose Y", getTargetPoseRobotRelative().get().getY());
            SmartDashboard.putNumber("LL From Pose Z", getTargetPoseRobotRelative().get().getZ());
        } catch (Exception e) {}
    }                                                                                        
}
