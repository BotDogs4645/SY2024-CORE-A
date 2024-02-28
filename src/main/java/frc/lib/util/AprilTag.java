// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AdvanceToTarget;

// The AprilTag class, harnessed in order to
// calculate and execute various 'vision'-related
// processes within FIRST Team 4645, the Chicago Style 
// Bot Dogs' robot for the FIRST Robotics 2024 competition.

public class AprilTag {

    public static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("limelight");

    public final AdvanceToTarget advanceToTargetInstance;

    public AprilTag(AdvanceToTarget advanceToTargetInstance) {
      this.advanceToTargetInstance = advanceToTargetInstance;
    }


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
        if (getDirectDistance(targetPosition).isPresent()) {
        //   System.out.println("Current spatial distance to primary target: " + getDirectDistance(targetPosition).get());
            determineTargetRotationalOffset(Optional.of(targetPos().get().getTranslation()));
            advanceToTargetInstance.specifyTarget(targetPosition.get());
        }
      } else {
        System.out.println("No Limelight target.");
      }
    }

  // Fetches, calculates, and returns the offset from the robot to the current primary apriltag target.

  public Optional<Transform3d> targetPos() {
        Optional<double[]> targetPose = Optional.of(entry("targetpose_robotspace").get().getDoubleArray(new double[0]));
        // Optional<double[]> targetPose = Optional.of(new double[] {-5, 0, 0, 0, 0, 0});
        

        if (targetPose.isPresent()) {
          Transform3d transform = new Transform3d(
              new Translation3d(targetPose.get()[0], targetPose.get()[1], targetPose.get()[2]),
              new Rotation3d(targetPose.get()[3], targetPose.get()[4], targetPose.get()[5])
          );
          if(transform.getTranslation().getNorm() < 0.0001) {
            return Optional.empty();
          } else {
            return Optional.of(transform);
          }
        } else {
          return Optional.empty();
        }
    };

  // Calculates the distance between the robot's current estimated 
  // position and the one of the primary target apriltag.

  public Optional<Double> getDirectDistance(Optional<Transform3d> targetVector) {

    if (targetVector.isPresent()) {
      return Optional.of(Math.sqrt(Math.pow(targetVector.get().getX(), 2) + Math.pow(targetVector.get().getY(), 2) + Math.pow(targetVector.get().getZ(), 2)));
    } else {
      return Optional.empty();
    }
  }

  public Optional<Double> getDualDimensionalDistance(Optional<double[]> originPosition, Optional<double[]> targetPosition) {
    if (originPosition.isPresent() && targetPosition.isPresent()) {
      return Optional.of(Math.sqrt(Math.pow(targetPosition.get()[1] - originPosition.get()[1], 2) + Math.pow(targetPosition.get()[0] - originPosition.get()[0], 2)));
    } else {
      return Optional.empty();
    }
  }

  public boolean validTargetInput(Optional<Translation3d> targetPosition) {
    Optional<Transform3d> currentRobotPosition = determinePosition();

    if (targetPosition.isPresent() && currentRobotPosition.isPresent() && currentRobotPosition.get().getX() != targetPosition.get().getX() && currentRobotPosition.get().getY() != targetPosition.get().getY()) {
      return true;
    } else {
      return false;
    }
  }

  // Determines the robot's position on the playing 
  // field, through the use of limelight's vector distance API.

  public Optional<Transform3d> determinePosition() {
    Optional<Transform3d> relativePositionalData = targetPos();

    Optional<Long> aprilTagIdentifier = Optional.of(entry("tid").get().getInteger(-1));

    if (aprilTagIdentifier.isEmpty() || aprilTagIdentifier.get() > 16) {
        return Optional.empty();
    }
    
    Transform3d currentAprilTagLocation = Constants.Limelight.APRILTAGS.get(aprilTagIdentifier);

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

public Optional<double[]> determineTargetRotationalOffset(Optional<Translation3d> targetPosition) {
        // Optional<Transform3d> currentRelativePosition = determinePosition();
        Optional<Transform3d> currentRelativePosition = Optional.of(new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
        ));

        if ((targetPosition.isEmpty() || currentRelativePosition.isEmpty()) && !validTargetInput(targetPosition)) {
          return Optional.empty();
        }

        double xAxisOffset = currentRelativePosition.get().getX() - targetPosition.get().getX();
        double yAxisOffset = currentRelativePosition.get().getY() - targetPosition.get().getY();
        double xAngularOffset = (Math.atan2(yAxisOffset, xAxisOffset) * (180 / Math.PI)) - currentRelativePosition.get().getRotation().getX();
      
        double yAngularOffset = Math.asin((targetPosition.get().getZ() - currentRelativePosition.get().getZ()) / getDualDimensionalDistance(Optional.of(new double[] {currentRelativePosition.get().getX(), currentRelativePosition.get().getY()}), Optional.of(new double[] {targetPosition.get().getX(), targetPosition.get().getY()})).get());

        if (Double.isNaN(xAngularOffset) || Double.isNaN(yAngularOffset)) {
          return Optional.empty();
        }

        System.out.println("Target X Offset: " + xAngularOffset + ", Target Y Offset " + yAngularOffset);

        return Optional.of(new double[] {xAngularOffset, yAngularOffset});
    }

    /**
     * Returns the height difference between the robot and the target, if a target
     * is detected.
     * 
     * @return an OptionalDouble representing the absolute height difference to the
     *         target,
     *         or an empty OptionalDouble if no target is detected.
     */
    public Optional<Double> getHeightDifferenceToTarget() {
      // still need to implement the offsets between the actual target and the
      // apriltag
      if (targetPos().isPresent()) {
        double heightDifference = targetPos().get().getY();
        return Optional.of(Math.abs(heightDifference));
      } else {
        return Optional.empty();
      }
    }

    /**
     * Returns the vertical velocity component needed for the note to reach the
     * target, if a
     * target is detected.
     * 
     * @return an OptionalDouble representing the vertical velocity of the target,
     *         or empty if no target is detected
     */
    public Optional<Double> getVerticalVelocity() {
      if (targetPos().isPresent()) {
        return Optional.of(Math.sqrt(
            2 * Constants.Launcher.gravityAcceleration * getHeightDifferenceToTarget().get()));

      } else {
        return Optional.empty();
      }
    }

    /**
     * Calculates the time it takes for the note to travel to the target, if a
     * target is detected.
     * 
     * @return An OptionalDouble representing the time to travel, or empty if there
     *         is no target.
     */
    public Optional<Double> getTimeToTravel() {
      if (targetPos().isPresent()) {
        double timeToTravel = getVerticalVelocity().get() / Constants.Launcher.gravityAcceleration;
        return Optional.of(timeToTravel);
      }
      return Optional.empty();
    }


    /**
     * Returns the horizontal velocity component needed for the note to reach the
     * target, if a
     * target is detected.
     * 
     * @return an OptionalDouble representing the vertical velocity of the target,
     *         or empty if no target is detected
     */
    public Optional<Double> getHorizontalVelocity() {
      if (!targetPos().isPresent() && getHeightDifferenceToTarget().isPresent()) {
        return Optional.empty();
      }
      //TODO: Use a new method to find the planar 2D distance to the target instead of getDirectDistance(), so that it disregards height.
      return Optional.of(targetPos().map(transform -> Math.hypot(transform.getX(), transform.getY())).get() / getTimeToTravel().get());
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
}