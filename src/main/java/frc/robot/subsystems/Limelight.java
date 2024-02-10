package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    public static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("limelight");


    private double x = Double.NaN;
    private double y = Double.NaN;

    private OptionalDouble distanceToTarget = OptionalDouble.empty();
    private OptionalDouble ty = OptionalDouble.empty();
    private OptionalDouble tx = OptionalDouble.empty();
    private boolean hasTarget;

    // private AprilTag target = new AprilTag();


    // Fetches and returns the NetworkTable entry of the parameter provided in the method call.

    public Optional<NetworkTableEntry> entry(String key) {
        return Optional.of(TABLE.getEntry(key));
    }

    // Fetches, organizes, and returns the offset from the robot to the current primary apriltag target.

    public Optional<Transform3d> targetPos() {
        // Optional<double[]> targetPose = Optional.of(entry("targetpose_robotspace").get().getDoubleArray(new double[0]));
        Optional<double[]> targetPose = Optional.of(LimelightHelpers.getTargetPose_RobotSpace(""));

        if (targetPose.isPresent()) {
            return Optional.of(new Transform3d(
                new Translation3d(targetPose.get()[0], targetPose.get()[1], targetPose.get()[2]),
                new Rotation3d(targetPose.get()[3], targetPose.get()[4], targetPose.get()[5])
            ));
        } else {
            return Optional.empty();
        }
    };


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
        if (hasTarget) {
            return OptionalDouble.of((Constants.Vision.LimelightOffsetZ + Constants.APRILTAGS.get(getTagID().getAsInt()).getZ()) / Math.tan(ty.getAsDouble() + Constants.Vision.LimelightAngleDegrees));
        } else {
            return OptionalDouble.empty();
        }
    }

    // Calculates the distance between the robot's current estimated 
    // position and the one of the primary target apriltag.

    public Optional<Double> getDistanceToTarget(Optional<Transform3d> targetPosition) {
        if (targetPosition.isPresent()) {
            return Optional.of(Math.sqrt(Math.pow(targetPosition.get().getX(), 2) + Math.pow(targetPosition.get().getY(), 2) + Math.pow(targetPosition.get().getZ(), 2)));
        } else if (targetPos().isPresent()) {
            Optional<Transform3d> fallbackTargetPosition = targetPos();
            
            return Optional.of(Math.sqrt(Math.pow(fallbackTargetPosition.get().getX(), 2) + Math.pow(fallbackTargetPosition.get().getY(), 2) + Math.pow(fallbackTargetPosition.get().getZ(), 2)));
        } else {
            return Optional.empty();
        }
      }
    
    public Optional<Pose3d> getTargetPoseRobotRelative() {
        if (hasTarget) {
            return Optional.of(LimelightHelpers.getTargetPose3d_RobotSpace(""));
        } else {
            return Optional.empty();
        }
    }

    // The method exhibited below executes periodically while querying 
    // the primary target apriltag's current position and printing the
    // distance to such in the console.

    @Override
    public void periodic() {
        // hasTarget = LimelightHelpers.getTV("");
        // if (hasTarget) {
        //     distanceToTarget = OptionalDouble.of(Math.hypot(
        //         getDistanceToTargetPlane().getAsDouble(),
        //         getDistanceToTargetPlane().getAsDouble() * Math.tan(ty.getAsDouble())
        //     ));
        // }
        Optional<Transform3d> targetPosition = targetPos();
  
        if (targetPosition.isPresent()) {
          // System.out.printf("Target position: {x: %.3f, y: %.3f, z: %.3f}\n", t.getX(), t.getY(), t.getZ());
          if (getDistanceToTarget(targetPosition).isPresent()) {
            System.out.println("Current spatial distance to primary target: " + getDistanceToTarget(targetPosition).get());
          }
        } else {
          System.out.println("No Limelight target.");
        }
      }
}
