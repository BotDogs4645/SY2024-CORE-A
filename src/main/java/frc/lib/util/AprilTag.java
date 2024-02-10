// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.Optional;

import javax.swing.text.html.Option;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;

// The AprilTag class, harnessed in order to
// calculate and execute various 'vision'-related
// processes within FIRST Team 4645, the Chicago Style 
// Bot Dogs' robot for the FIRST Robotics 2024 competition.

public class AprilTag {

  // Determines the robot's position on the playing 
  // field, through the use of limelight's vector distance API.

  public Optional<Transform3d> determinePosition(Limelight limelightClassInstance) {
    Optional<Transform3d> relativePositionalData = limelightClassInstance.targetPos();

    Optional<Long> aprilTagIdentifier = Optional.of(limelightClassInstance.entry("tid").get().getInteger(-1));

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

    Optional<Transform3d> currentRelativePosition = Optional.of(new Transform3d (
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

  public Optional<Rotation2d> determineTargetRotationalOffset(Limelight limelightClassInstance, Optional<Translation3d> targetPosition) {
        Optional<Transform3d> currentRelativePosition = determinePosition(limelightClassInstance);

        // if (currentRelativePosition.isEmpty()) {
        //     System.out.printf("An apriltag is currently not within view, and therefore, we are unable to calculate the positional offset of the target located at {X: %.3f, Y: %.3f, Z: %.3f}\n.", targetPosition.get().getX(), targetPosition.get().getY(), targetPosition.get().getZ());     
        //     return Optional.empty();
        // }

        if ((currentRelativePosition.get().getX() == targetPosition.get().getX()) && (currentRelativePosition.get().getY() == targetPosition.get().getY())) {
            System.out.println("The robot is currently located at the same position as the apriltag target.");
            return Optional.empty();
        }

        double targetXAxisOffset = Math.toDegrees(Math.asin(Math.abs(currentRelativePosition.get().getX() - targetPosition.get().getX()) / Math.sqrt(Math.pow(currentRelativePosition.get().getX() - targetPosition.get().getX(), 2) + Math.pow(currentRelativePosition.get().getY() - targetPosition.get().getY(), 2))));
        double targetZAxisOffset;

        if (currentRelativePosition.get().getZ() == targetPosition.get().getZ()) {
            targetZAxisOffset = 0;
        } else {
            targetZAxisOffset = Math.toDegrees(Math.asin(Math.abs(currentRelativePosition.get().getZ() - targetPosition.get().getZ()) / (targetPosition.get().getDistance(currentRelativePosition.get().getTranslation()))));
        }

        if ((currentRelativePosition.get().getX() > targetPosition.get().getX() && ((270 < currentRelativePosition.get().getRotation().getX()) || (currentRelativePosition.get().getRotation().getX() < 90))) || (currentRelativePosition.get().getY() > targetPosition.get().getY() && ((270 < currentRelativePosition.get().getRotation().getY()) || (currentRelativePosition.get().getRotation().getY() < 90)))) {
            targetXAxisOffset += 180;
        }

        System.out.println("Target X Offset: " + targetXAxisOffset + ", Target Z Offset: " + targetZAxisOffset);
        
        return Optional.of(new Rotation2d(targetXAxisOffset, targetZAxisOffset));
  }
}