// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.io.OptionalDataException;
import java.lang.annotation.Target;
import java.util.Optional;
import java.util.OptionalDouble;

import javax.swing.text.html.Option;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

// The AprilTag class, harnessed in order to
// calculate and execute various 'vision'-related
// processes within FIRST Team 4645, the Chicago Style 
// Bot Dogs' robot for the FIRST Robotics 2024 competition.

public class AprilTag {

    public static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("limelight");

    // Fetches and returns the NetworkTable entry of the parameter provided in the method call.

    public Optional<NetworkTableEntry> entry(String key) {
        return Optional.of(TABLE.getEntry(key));
    }

    // Executes periodically while quering the primary target apriltag's 
    // current position and printing the distance to such in the console.

    public void aprilTagPeriodic() {
      Optional<Transform3d> targetPosition = targetPos();

      if (targetPosition.isPresent()) {
        // System.out.printf("Target position: {x: %.3f, y: %.3f, z: %.3f}\n", t.getX(), t.getY(), t.getZ());
        if (getDirectDistance().isPresent()) {
        //   System.out.println("Current spatial distance to primary target: " + getDirectDistance().get());
            determineTargetRotationalOffset(Optional.of(targetPos().get().getTranslation()));
        }
      } else {
        System.out.println("No Limelight target.");
      }
    }

  // Fetches, calculates, and returns the offset from the robot to the current primary apriltag target.

  public Optional<Transform3d> targetPos() {
        // Optional<double[]> targetPose = Optional.of(entry("targetpose_robotspace").get().getDoubleArray(new double[0]));
        Optional<double[]> targetPose = Optional.of(new double[] {5, 10, 0, 0, 0, 0});
        

        if (targetPose.isPresent()) {
          return Optional.of(new Transform3d(
              new Translation3d(targetPose.get()[0], targetPose.get()[1], targetPose.get()[2]),
              new Rotation3d(targetPose.get()[3], targetPose.get()[4], targetPose.get()[5])
          ));
        } else {
          return Optional.empty();
        }
    };

  // Calculates the distance between the robot's current estimated 
  // position and the one of the primary target apriltag.

  public Optional<Double> getDirectDistance() {
    Optional<Transform3d> targetVector = targetPos();

    if (targetVector.isPresent()) {
      return Optional.of(Math.sqrt(Math.pow(targetVector.get().getX(), 2) + Math.pow(targetVector.get().getY(), 2) + Math.pow(targetVector.get().getZ(), 2)));
    } else {
      return Optional.empty();
    }
  }

  // Returns the 

  public Optional<Integer> determineRotationalSector(double currentXOrientation) {

    if (Optional.of(currentXOrientation).isEmpty()) {
        return Optional.empty();
    }

    Optional<Integer> rotationalSector = Optional.empty();
    double secondaryXOrientation = currentXOrientation;
    
    if (currentXOrientation == 0) {
        secondaryXOrientation = 360;
    } else if (currentXOrientation == 360) {
        secondaryXOrientation = 0;
    }

    if ((currentXOrientation > 270 || secondaryXOrientation > 270) && 90 > currentXOrientation) {
        rotationalSector = Optional.of(0);
    } else if ((currentXOrientation > 0 || secondaryXOrientation > 0) && currentXOrientation < 180) {
        rotationalSector = Optional.of(1);
    } else if (currentXOrientation > 90 && currentXOrientation < 270) {
        rotationalSector = Optional.of(2);
    } else if (currentXOrientation > 180 && currentXOrientation < 360) {
        rotationalSector = Optional.of(3);
    }

    return rotationalSector;
  }

  // Determines the robot's position on the playing 
  // field, through the use of limelight's vector distance API.

  public Optional<Transform3d> determinePosition() {
    Optional<Transform3d> relativePositionalData = targetPos();

    Optional<Long> aprilTagIdentifier = Optional.of(entry("tid").get().getInteger(-1));

    if (aprilTagIdentifier.isEmpty() || aprilTagIdentifier.get() > 16) {
        return Optional.empty();
    }
    
    Transform3d currentAprilTagLocation = Constants.APRILTAGS.get(aprilTagIdentifier);

    double[] currentRelativeLocation = {
        currentAprilTagLocation.getX() + relativePositionalData.get().getX(), 
        currentAprilTagLocation.getY() + relativePositionalData.get().getY(), 
        currentAprilTagLocation.getZ() + relativePositionalData.get().getZ()
    };

    double[] currentRelativeRotation = {
        currentAprilTagLocation.getRotation().getX() + relativePositionalData.get().getRotation().getX(),
        currentAprilTagLocation.getRotation().getY() + relativePositionalData.get().getRotation().getY(),
        currentAprilTagLocation.getRotation().getZ() + relativePositionalData.get().getRotation().getZ(),
    };

    if (currentRelativeRotation[0] == 360) {
        currentRelativeRotation[0] = 0;
    }

    Optional<Transform3d> currentRelativePosition = Optional.of(new Transform3d(
        new Translation3d(currentRelativeLocation[0], currentRelativeLocation[1], currentRelativeLocation[2]),
        new Rotation3d(currentRelativeRotation[0], currentRelativeRotation[1], currentRelativeRotation[2])
    ));

    System.out.printf("Current relative robot position: {X: %0.3f, Y: %0.3f, Z: %0.3f}\n", currentRelativePosition.get().getX(), currentRelativePosition.get().getY(), currentRelativePosition.get().getZ());
    System.out.printf("Current relative robot orientation: {X: %0.3f, Y: %0.3f, Z: %0.3f}\n", currentRelativePosition.get().getRotation().getX(), currentRelativePosition.get().getRotation().getY(), currentRelativePosition.get().getRotation().getZ());

    return currentRelativePosition;
}

  // Calculates the relative rotational offset to the 
  // given "targetPosition," based off of the current 
  // orientation of the robot within 3D space.

public Optional<Rotation2d> determineTargetRotationalOffset(Optional<Translation3d> targetPosition) {
        // Optional<Transform3d> currentRelativePosition = determinePosition();
        Optional<Transform3d> currentRelativePosition = Optional.of(new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
        ));

        if (currentRelativePosition.isEmpty()) {
            return Optional.empty();
        }

        double currentXOrientation = currentRelativePosition.get().getRotation().getX();
        double targetXAxisOffset;
        double targetZAxisOffset;
        int rotationalSector = determineRotationalSector(currentXOrientation).get();

        // if (currentRelativePosition.isEmpty()) {
        //     System.out.printf("An apriltag is currently not within view, and therefore, we are unable to calculate the positional offset of the target located at {X: %.3f, Y: %.3f, Z: %.3f}\n.", targetPosition.get().getX(), targetPosition.get().getY(), targetPosition.get().getZ());     
        //     return Optional.empty();
        // }

        if ((currentRelativePosition.get().getX() == targetPosition.get().getX()) && (currentRelativePosition.get().getY() == targetPosition.get().getY())) {
            System.out.println("The robot is currently located at the same position as the apriltag target.");
            return Optional.empty();
        }
        
        targetXAxisOffset = Math.toDegrees(Math.asin(Math.abs(currentRelativePosition.get().getX() - targetPosition.get().getX()) / Math.sqrt(Math.pow(currentRelativePosition.get().getX() - targetPosition.get().getX(), 2) + Math.pow(currentRelativePosition.get().getY() - targetPosition.get().getY(), 2)))) + Constants.Vision.rotationalSectorOffsets.get(rotationalSector);

        if (currentRelativePosition.get().getZ() == targetPosition.get().getZ()) {
            targetZAxisOffset = 0;
        } else {
            targetZAxisOffset = Math.toDegrees(Math.asin(Math.abs(currentRelativePosition.get().getZ() - targetPosition.get().getZ()) / (targetPosition.get().getDistance(currentRelativePosition.get().getTranslation()))));
        }


        System.out.println("Original Target X Offset: " + (targetXAxisOffset - Constants.Vision.rotationalSectorOffsets.get(rotationalSector)));
        // System.out.println("Target X Offset: " + targetXAxisOffset + ", Target Z Offset: " + targetZAxisOffset);
        
        return Optional.of(new Rotation2d(targetXAxisOffset, targetZAxisOffset));
    }
}