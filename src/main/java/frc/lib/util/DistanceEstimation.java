package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class DistanceEstimation {

    public Pose2d estimateNoteDistance(double xAngularOffset, double yAngularOffset) {
        if (Constants.DistanceEstimation.baseLimelightYRotation + yAngularOffset >= 90) {
            yAngularOffset = 89.9444444444444 - Constants.DistanceEstimation.baseLimelightYRotation;
        }

        double yOffset = Constants.DistanceEstimation.noteZOffset * Math.tan(Math.toRadians(Math.abs(Constants.DistanceEstimation.baseLimelightYRotation + yAngularOffset)));
        double xOffset = yOffset * Math.tan(Math.toRadians(Math.abs(xAngularOffset)));

        return new Pose2d(new Translation2d(xOffset, yOffset), new Rotation2d(xAngularOffset, Constants.DistanceEstimation.baseLimelightYRotation + yAngularOffset));
    }

    public double calculateLimelightPitch(double currentNoteYDistance, double currentYOffset) {        
        return Math.atan(currentNoteYDistance / Constants.DistanceEstimation.noteZOffset) - currentYOffset;
    }
}