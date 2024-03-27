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
import frc.robot.Constants.Vision;
import frc.robot.LimelightHelpers;

/**
 * The FrontLimelight subsystem.
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

    /**
     * Calculates the vertical velocity vector component required to reach the target.
     * @return vertical velocity component in m/s
     */
    public OptionalDouble getVerticalVelocity(double verticalDistance) {
        return OptionalDouble.of(Math.sqrt(2 * Constants.Launcher.gravityAcceleration * verticalDistance));
    }

    /**
     * Calculates the horizontal velocity vector component required to reach the target.
     * @return horizontal velocity component in m/s
     */
    public OptionalDouble getHorizontalVelocity(double verticalDistance, double horizontalDistance) {
        if (getHorizontalVelocity(verticalDistance, horizontalDistance).isEmpty()) {
            return OptionalDouble.empty();
        }

        double timeToTravel = (-1 * getHorizontalVelocity(verticalDistance, horizontalDistance).getAsDouble() + Math.sqrt(
                Math.pow(getHorizontalVelocity(verticalDistance, horizontalDistance).getAsDouble(), 2) + 2 * Constants.Launcher.gravityAcceleration * verticalDistance))
                / (Constants.Launcher.gravityAcceleration);
        return OptionalDouble.of(horizontalDistance / timeToTravel);
    }

    /**
     * Calculates the launch velocity required to reach the target, in RPM, when launching at the calculated angle from getLaunchAngle().
     * @return launch velocity in RPM
     */
    public OptionalDouble getLaunchVelocity(double verticalDistance, double horizontalDistance) {
        if (getVerticalVelocity(verticalDistance).isEmpty() || getHorizontalVelocity(verticalDistance, horizontalDistance).isEmpty()) {
            return OptionalDouble.empty();
        }

        return toRPM(Math.sqrt(Math.pow(getVerticalVelocity(verticalDistance).getAsDouble(), 2) + Math.pow(getHorizontalVelocity(verticalDistance, horizontalDistance).getAsDouble(), 2)));
    }

    /**
     * Calculates the launch angle required to reach the target, in radians.
     * @return launch angle in radians
     */
    public OptionalDouble getLaunchAngle(double verticalDistance, double horizontalDistance) {
        if (getVerticalVelocity(verticalDistance).isEmpty() || getHorizontalVelocity(verticalDistance, horizontalDistance).isEmpty()) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(Math.toDegrees(Math.atan(getVerticalVelocity(verticalDistance).getAsDouble() / getHorizontalVelocity(verticalDistance, horizontalDistance).getAsDouble())));
    }

    /**
     * Converts a velocity in m/s to RPM using the launcher wheel radius from Constants.Launcher.launcherWheelRadius.
     * @param velocity velocity in m/s
     * @return velocity in RPM
     */
    public OptionalDouble toRPM(double velocity) {
        return OptionalDouble.of(velocity * 60 / (2 * Math.PI * Constants.Launcher.launcherWheelRadius));
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
