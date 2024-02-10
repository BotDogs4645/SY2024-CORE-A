package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.AprilTag;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/**
 * The Limelight subsystem.
 * 
 * Limelight documentation: https://limelightvision.io/
 * 
 * We use the Limelight for location estimation with AprilTags.
 */
public class Limelight extends SubsystemBase {

    private double x = Double.NaN;
    private double y = Double.NaN;

    private OptionalDouble distanceToTarget = OptionalDouble.empty();
    private OptionalDouble ty = OptionalDouble.empty();
    private OptionalDouble tx = OptionalDouble.empty();
    private boolean hasTarget;

    private AprilTag target = new AprilTag();

    public OptionalDouble getLateralAngleToTarget() {
        return tx;
    }

    public OptionalDouble getVerticalAngleToTarget() {
        return ty;
    }

    public OptionalInt getTagID() {
        if (hasTarget) {
            return OptionalInt.of((int) LimelightHelpers.getFiducialID(""));
        } else {
            return OptionalInt.empty();
        }
    }

    public OptionalDouble getDistanceToTargetPlane() {
        Optional<Pose3d> relative = getTargetPoseRobotRelative();
        if (relative.isPresent()) {
            return OptionalDouble.of(relative.get().getY());
        } else {
            return OptionalDouble.empty();
        }
        // if (hasTarget) {
        // return OptionalDouble
        // .of((Constants.Vision.LimelightOffsetZ +
        // Constants.APRILTAGS.get(getTagID().getAsInt()).getZ())
        // / Math.tan(ty.getAsDouble() + Constants.Vision.LimelightAngleDegrees));
        // } else {
        // return OptionalDouble.empty();
        // }
    }

    public OptionalDouble getDistanceToTarget() {
        return distanceToTarget;
    }

    public Optional<Pose3d> getTargetPoseRobotRelative() {
        if (hasTarget) {
            return Optional.of(LimelightHelpers.getTargetPose3d_RobotSpace(""));
        } else {
            return Optional.empty();
        }
    }

    public OptionalDouble getVerticalVelocity() {
        if (hasTarget) {
            double heightDifference = Constants.APRILTAGS.get(getTagID().getAsInt()).getZ()
                    - Constants.Launcher.launcherHeight;
            return OptionalDouble.of(Math.sqrt(
                    2 * Constants.Launcher.gravityAcceleration * (Constants.APRILTAGS.get(getTagID().getAsInt()).getZ())
                            - Constants.Launcher.launcherHeight));
        } else {
            return OptionalDouble.empty();
        }
    }

    public OptionalDouble getHorizontalVelocity() {
        if (!hasTarget) {
            return OptionalDouble.empty();
        }

        double timeToTravel = (-1 * getVerticalVelocity().getAsDouble() + Math.sqrt(
                Math.pow(getVerticalVelocity().getAsDouble(), 2)
                        + 2 * Constants.Launcher.gravityAcceleration
                                * (Constants.APRILTAGS.get(getTagID().getAsInt()).getZ())
                        - Constants.Launcher.launcherHeight))
                / (Constants.Launcher.gravityAcceleration);
        return OptionalDouble.of(getDistanceToTargetPlane().getAsDouble() / timeToTravel);
    }

    public OptionalDouble getLaunchVelocity() {
        if (hasTarget) {
            return OptionalDouble
                    .of(toRPM(Math.hypot(getHorizontalVelocity().getAsDouble(), getVerticalVelocity().getAsDouble())));
        }
        return OptionalDouble.empty();
    }

    public OptionalDouble getLaunchAngle() {
        if (hasTarget) {
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
        hasTarget = LimelightHelpers.getTV("");
        if (hasTarget) {
            distanceToTarget = OptionalDouble.of(Math.hypot(
                    getDistanceToTargetPlane().getAsDouble(),
                    getDistanceToTargetPlane().getAsDouble() * Math.tan(ty.getAsDouble())));
        }
    }

}
