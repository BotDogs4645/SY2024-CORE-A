package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;

import frc.robot.Constants.Vision;

/**
 * The BackLimelight subsystem.
 * 
 * Limelight documentation: https://limelightvision.io/
 * 
 * We use the Limelight for location estimation with AprilTags.
 */
public class BackLimelight extends SubsystemBase {

    public boolean hasTarget() {
        return (LimelightHelpers.getTA(Vision.BackLimelight.Name) > 0.1) ? true : false;
    }

    public Optional<Pose3d> getTargetPoseRelative() {
        try {
            return Optional.of(LimelightHelpers.getTargetPose3d_RobotSpace(Vision.BackLimelight.Name));             
        } catch (Exception e) {
            return Optional.empty();
        }
    }

    /** 
    * Calculates and @return(s) the relative rotational offset to the given 
    * "targetPosition," based off of Limelight's estimation on where the robot is 
    * currently positioned, as well as the location of the specified target.
    *
    * For a map of the game board and AprilTag positions, see
    * {@link https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf},
    * page 4.
    */
    public OptionalDouble determineTargetRotationalOffset() {
        return OptionalDouble.of(getTargetPoseRelative().get().getRotation().getX());
    }
}
