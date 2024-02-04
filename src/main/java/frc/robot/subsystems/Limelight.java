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
        if(hasTarget) {
            return OptionalInt.of((int) LimelightHelpers.getFiducialID(""));
        } else {
            return OptionalInt.empty();
        }
    }

    public OptionalDouble getDistanceToTargetPlane() {
        if(hasTarget) {
            return OptionalDouble.of((Constants.Vision.LimelightOffsetZ + Constants.APRILTAGS.get(getTagID().getAsInt()).getZ()) / Math.tan(ty.getAsDouble() + Constants.Vision.LimelightAngleDegrees));
        } else {
            return OptionalDouble.empty();
        }
    }

    public OptionalDouble getDistanceToTarget() {
        return distanceToTarget;
    }
    
    public Optional<Pose3d> getTargetPoseRobotRealative() {
        if(hasTarget) {
            return Optional.of(LimelightHelpers.getTargetPose3d_RobotSpace(""));
        } else {
            return Optional.empty();
        }
    }


    @Override
    public void periodic() {
        hasTarget = LimelightHelpers.getTV("");
        if(hasTarget) {
            distanceToTarget = OptionalDouble.of(Math.hypot(
                getDistanceToTargetPlane().getAsDouble(),
                getDistanceToTargetPlane().getAsDouble() * Math.tan(ty.getAsDouble())
            ));
        }
    }
    
}
