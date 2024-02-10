package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private OptionalDouble distanceToTarget = OptionalDouble.empty();
    private OptionalDouble ty = OptionalDouble.empty();
    private OptionalDouble tx = OptionalDouble.empty();

    private AprilTag target = new AprilTag();

    public boolean hasTarget() {
        return (LimelightHelpers.getTA("") > 0.1) ? true : false;
    }

    public OptionalDouble getLateralAngleToTarget() {
        return tx;
    }

    public OptionalDouble getVerticalAngleToTarget() {
        return ty;
    }

    public OptionalInt getTagID() {
        try {
            return OptionalInt.of((int) LimelightHelpers.getFiducialID(""));
        } catch (Exception e) {
            return OptionalInt.empty();
        }
    }

    public OptionalDouble getDistanceToTarget() {
        if (!ty.isPresent() && !hasTarget()) {
            return OptionalDouble.empty();
        }
        double d = (
            (Constants.Vision.LimelightOffsetZ - Constants.APRILTAGS.get(
                (int) LimelightHelpers.getFiducialID("")).getZ()
            ) / Math.tan(Constants.Vision.LimelightAngleDegrees + ty.getAsDouble())
        ); 

        return OptionalDouble.of(d);
    }

  
    public Optional<Pose3d> getTargetPoseRobotRealative() {
        try {
            return Optional.of(LimelightHelpers.getTargetPose3d_RobotSpace(""));             
        } catch (Exception e) {
            return Optional.empty();
        }
    }


    public OptionalDouble getHeightDifferenceToObjective() {
        //will only return height difference if the target is in the list of shooter targets
        if (hasTarget() && Constants.shooterAprilTags.contains((int) LimelightHelpers.getFiducialID(""))){
            double heightDifference = Constants.Vision.LimelightOffsetZ -
                    Constants.APRILTAGS.get((int) LimelightHelpers.getFiducialID("")).getZ() +
                    Constants.APRILTAG_OBJECTIVE_OFFSETS.get((int) LimelightHelpers.getFiducialID("")).getZ();
            return OptionalDouble.of(heightDifference);
        } else {
            return OptionalDouble.empty();
        }
    }
    

    public OptionalDouble getVerticalVelocity() {
        if (hasTarget() && getHeightDifferenceToObjective().isPresent()) {
            return OptionalDouble.of(Math.sqrt(
                    2 * Constants.Launcher.gravityAcceleration * getHeightDifferenceToObjective().getAsDouble()));
        } else {
            return OptionalDouble.empty();
        }
    }

    public OptionalDouble getHorizontalVelocity() {
        if (!hasTarget()  && getHeightDifferenceToObjective().isPresent()) {
            return OptionalDouble.empty();
        }

        double timeToTravel = (-1 * getVerticalVelocity().getAsDouble() + Math.sqrt(
                Math.pow(getVerticalVelocity().getAsDouble(), 2)
                        + 2 * Constants.Launcher.gravityAcceleration
                                * getHeightDifferenceToObjective().getAsDouble()))
                / (Constants.Launcher.gravityAcceleration);
        return OptionalDouble.of(getDistanceToTarget().getAsDouble() / timeToTravel);
    }

    public OptionalDouble getLaunchVelocity() {
        if (hasTarget() && getHeightDifferenceToObjective().isPresent()){
            return OptionalDouble
                    .of(toRPM(Math.hypot(getHorizontalVelocity().getAsDouble(), getVerticalVelocity().getAsDouble())));
        }
        return OptionalDouble.empty();
    }

    public OptionalDouble getLaunchAngle() {
        if (hasTarget() && getHeightDifferenceToObjective().isPresent()){
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
            tx = OptionalDouble.of(LimelightHelpers.getTX(""));
            ty = OptionalDouble.of(LimelightHelpers.getTY(""));
            distanceToTarget = getDistanceToTarget();
        } else {
            tx = OptionalDouble.empty();
            ty = OptionalDouble.empty();
            distanceToTarget = OptionalDouble.empty();
        }
        SmartDashboard.putNumber("LL Distance", getDistanceToTarget().orElse(-1));
        try {
            SmartDashboard.putNumber("LL From Pose X", getTargetPoseRobotRealative().get().getX());
            SmartDashboard.putNumber("LL From Pose Y", getTargetPoseRobotRealative().get().getY());
            SmartDashboard.putNumber("LL From Pose Z", getTargetPoseRobotRealative().get().getZ());
        } catch (Exception e) {}
    }                                                                                        
}
