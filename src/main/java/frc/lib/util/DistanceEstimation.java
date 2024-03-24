package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class DistanceEstimation {

    public Pose2d estimateNoteDistance(double xAngularOffset, double yAngularOffset) {
        double yOffset = Constants.DistanceEstimation.noteZOffset * Math.tan(yAngularOffset);
        double xOffset = yOffset * Math.tan(xAngularOffset);

        return new Pose2d(new Translation2d(xOffset, yOffset), new Rotation2d(xAngularOffset, yAngularOffset));
    }
}
