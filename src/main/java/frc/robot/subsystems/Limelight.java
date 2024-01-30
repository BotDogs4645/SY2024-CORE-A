package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

    public static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("limelight");

    public static NetworkTableEntry entry(String key) {
        return TABLE.getEntry(key);
    }

    public boolean hasTarget() {
        return targetPos() != null;
    }

    public static Transform3d targetPos() {
        var targetPose = entry("targetpose_robotspace").getDoubleArray(new double[0]);
        if (targetPose.length != 6 || targetPose[2] < 1E-6) return null;
        
        return new Transform3d(
            new Translation3d(targetPose[0], targetPose[1], targetPose[2]),
            new Rotation3d(targetPose[3], targetPose[4], targetPose[5])
        );
    };

    // The method declared below exists as a way in which one is able to determine the robot's current location and orientation within the playing field shown within the FIRST Robotics 2024 competition, Crescendo.

    public static Transform3d determineRelativePosition() {
        var relativePositionalData = targetPos();
        var aprilTagIdentifier = entry("tid").getInteger(-1);
        if (aprilTagIdentifier == -1 || aprilTagIdentifier > 16) {
            return null;
        }
        Transform3d currentAprilTagLocation = Constants.APRILTAGS.get(aprilTagIdentifier);

        double[] currentRelativeLocation = {
            currentAprilTagLocation.getX() + relativePositionalData.getX(), 
            currentAprilTagLocation.getY() + relativePositionalData.getY(), 
            currentAprilTagLocation.getZ() + relativePositionalData.getZ()
        };

        double[] currentRelativeRotation = {
            currentAprilTagLocation.getRotation().getX() + relativePositionalData.getRotation().getX(),
            currentAprilTagLocation.getRotation().getY() + relativePositionalData.getRotation().getY(),
            currentAprilTagLocation.getRotation().getZ() + relativePositionalData.getRotation().getZ(),
        };

        Transform3d currentRelativePosition = new Transform3d (
            new Translation3d(currentRelativeLocation[0], currentRelativeLocation[1], currentRelativeLocation[2]),
            new Rotation3d(currentRelativeRotation[0], currentRelativeRotation[1], currentRelativeRotation[2])
        );

        System.out.printf("Current relative robot position: {X: %0.3f, Y: %0.3f, Z: %0.3f}\n", currentRelativePosition.getX(), currentRelativePosition.getY(), currentRelativePosition.getZ());
        System.out.printf("Current relative robot orientation: {X: %0.3f, Y: %0.3f, Z: %0.3f}\n", currentRelativePosition.getRotation().getX(), currentRelativePosition.getRotation().getY(), currentRelativePosition.getRotation().getZ());

        return currentRelativePosition;
    }

    // The method declared below exists as a calculation tool for the distance between two existences within 2D space.

    public double determineSpatialDistance(double lengthOne, double lengthTwo) {
        double spatialDistance = Math.sqrt((lengthOne * lengthOne) + (lengthTwo * lengthTwo));

        return spatialDistance;
    }

    // The method declared below exists as a determiner of the rotational offset of the robot's current position to a given target within the playing field of Crescendo, FIRST Robotics' competition within the year of 2024.

    public Rotation2d determineTargetRotationalOffset(Translation3d targetPosition) {
        // Transform3d currentRelativePosition = determineRelativePosition();
        Transform3d currentRelativePosition = new Transform3d (
            new Translation3d(1, 1, 1),
            new Rotation3d(1, 1, 1)
        );

        if (currentRelativePosition == null) {
            System.out.printf("An apriltag is currently not within view, and therefore, we are unable to calculate the positional offset of the target located at {X: %.3f, Y: %.3f, Z: %.3f}\n.", targetPosition.getX(), targetPosition.getY(), targetPosition.getZ()); // As of 1/09/24, I am unsure as to why this line of code is throwing an error in the console ("Unhandled exception: java.util.MissingFormatWidthException: %0.3f")... - Carver       
            return null;
        }

        if ((currentRelativePosition.getX() == targetPosition.getX()) && (currentRelativePosition.getY() == targetPosition.getY())) {
            System.out.println("As the given position is the one in which the robot is currently located, we are unfortunately, unable to calculate the rotational offset to said location.");
            return null;
        }

        double targetXAxisOffset = Math.toDegrees(Math.asin(Math.abs(currentRelativePosition.getX() - targetPosition.getX()) / Math.sqrt(Math.pow(currentRelativePosition.getX() - targetPosition.getX(), 2) + Math.pow(currentRelativePosition.getY() - targetPosition.getY(), 2))));
        double targetZAxisOffset;

        if (currentRelativePosition.getZ() == targetPosition.getZ()) {
            targetZAxisOffset = 0;
        } else {
            targetZAxisOffset = Math.toDegrees(Math.asin(Math.abs(currentRelativePosition.getZ() - targetPosition.getZ()) / (targetPosition.getDistance(currentRelativePosition.getTranslation()))));
        }

        System.out.println("Target X Offset: " + targetXAxisOffset + ", Target Z Offset: " + targetZAxisOffset);
        
        return new Rotation2d(targetXAxisOffset, targetZAxisOffset);
    }
}