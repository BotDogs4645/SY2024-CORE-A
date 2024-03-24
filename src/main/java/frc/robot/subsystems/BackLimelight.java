
package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DistanceEstimation;
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

    private final DistanceEstimation distanceEstimation;

    public BackLimelight(DistanceEstimation distanceEstimation) {
        this.distanceEstimation = distanceEstimation;
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(Vision.BackLimelight.Name);
    }

    /**
     * Fetches the rotational offset of the current note target, if it
     * is present, returning 'Optional.empty()' if such does not exist.
     * 
     * @return(s) the rotational offset to the current note target, 
     * or 'Optional.empty()' if such does not exist.
     */
    public Optional<double[]> getTargetInformation() {
        try {
            return Optional.of(new double[] {
                    LimelightHelpers.getTX(Vision.BackLimelight.Name),
                    LimelightHelpers.getTY(Vision.BackLimelight.Name),
            });
        } catch (Exception e) {
            return Optional.empty();
        }
    }

    public Optional<Pose2d> getEstimatedTargetPose() {
        Optional<double[]> targetInformation = getTargetInformation();

        if (targetInformation.isPresent()) {
            return Optional.of(distanceEstimation.estimateNoteDistance(targetInformation.get()[0], targetInformation.get()[1]));
        } else {
            return Optional.empty();
        }
    }
}
