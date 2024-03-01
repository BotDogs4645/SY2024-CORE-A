// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;

/**
* The AprilTag class, harnessed in order to calculate and execute various 
* 'vision'-related processes within FIRST Team 4645, the Chicago Style 
* Bot Dogs' robot for the FIRST Robotics 2024 competition, Crescendo.
*/
public class AprilTag {

  public static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("limelight");

  public final Limelight limelightInstance;

  public AprilTag(Limelight limelightInstance) {
    this.limelightInstance = limelightInstance;
  }

  public Optional<Transform3d> targetPos() {
    return limelightInstance.targetPos();
  }

  /**
   * Executes periodically while querying the primary apriltag target's current 
   * position, printing the distance to such to the console.
   */
  public void aprilTagPeriodic() {
    if (determinePosition().isEmpty() || targetPos().isEmpty()) {
      System.out.println("No Limelight target.");
    }

    Transform3d targetPosition = determinePosition().get().plus(targetPos().get());

    System.out.printf("Target position: {x: %.3f, y: %.3f, z: %.3f}\n", targetPosition.getX(), targetPosition.getY(), targetPosition.getZ());
  }

  /**
   * Calculates the distance between the robot's current estimated position
   * and that of the target specified in the method's parameters.
   * 
   * @param targetPosition - The position of the target in question, formatted 
   * through the use of Java's 'Transform3d' class.
   * 
   * @return(s) the distance from the robot to the target specified in the 
   * method's parameters.
   */
  public Optional<Double> getDirectDistance(Optional<Transform3d> targetPosition) {
    if (determinePosition().isPresent() && targetPosition.isPresent()) {
      return Optional.of(targetPos().get().getTranslation().getNorm());
    } else {
      return Optional.empty();
    }
  }

  /**
   * Calculates the distance, on a 2D plane, between two locations, as 
   * specified in the method's parameters.
   * 
   * @param originPosition - The position of the "primary" location, although 
   * the order of such and the "secondary" one does not affect, in any way,
   * the functionality of 
   * the method itself.
   * @param targetPosition - The position of the "secondary" location, although 
   * the order of such and the "primary" one does not affect, in any way, the 
   * functionality of the method itself.
   * 
   * @return(s) the distance which exists between the two locations specified 
   * within the method's parameters on a 2D plane.
   */
  public Optional<Double> getPlanarDistance(Translation2d originPosition, Translation2d targetPosition) {
    return Optional.of(Math.hypot(targetPosition.getX() - originPosition.getX(), targetPosition.getY() - originPosition.getY()));
  }

  /**
   * Determines whether the position of the target specified in the method's 
   * parameters is "suitable" for use in trajectory/path generation.
   * 
   * @param targetPosition - The position of the target in question.
   * 
   * @return(s) whether the position of the target specified in the method's 
   * parameters is "suitable" for use in trajectory/path generation.
   */
  public boolean validTargetInput(Optional<Translation3d> targetPosition) {
    Optional<Transform3d> currentRobotPosition = determinePosition();

    if (targetPosition.isPresent() && currentRobotPosition.isPresent() && currentRobotPosition.get().getX() != targetPosition.get().getX() && currentRobotPosition.get().getY() != targetPosition.get().getY()) {
      return true;
    } else {
      return false;
    }
  }

      /**
     * Estimates the position of the robot, using the Limelight's knowledge of
     * the position of each AprilTag in the game board.
     * If no AprilTags are deteted, this returns 'Optional.empty()'.
     * 
     * For a map of the game board and AprilTag positions, see
     * {@link https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf},
     * page 4.
     * 
     * @return(s) the robot's estimated position in the board.
     */
    public Optional<Transform3d> determinePosition() {
      Optional<Transform3d> targetOffset = targetPos();
      if (targetOffset.isEmpty()) return Optional.empty();

      Optional<Transform3d> originToTarget = Optional.of(Constants.Limelight.APRILTAGS.get(limelightInstance.targetId().get()));
      if (originToTarget.isEmpty()) return Optional.empty();

      Transform3d targetToRobot = targetOffset.get().inverse();

      Transform3d originToRobot = originToTarget.get().plus(targetToRobot);

      return Optional.of(originToRobot);
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
  public Optional<double[]> determineTargetRotationalOffset(Optional<Translation3d> targetPosition) {
        Optional<Transform3d> currentRelativePosition = determinePosition();

        if ((targetPosition.isEmpty() || currentRelativePosition.isEmpty()) && !validTargetInput(targetPosition)) {
          return Optional.empty();
        }

        double xAxisOffset = currentRelativePosition.get().getX() - targetPosition.get().getX();
        double yAxisOffset = currentRelativePosition.get().getY() - targetPosition.get().getY();
        double xAngularOffset = (Math.atan2(yAxisOffset, xAxisOffset) * (180 / Math.PI)) - currentRelativePosition.get().getRotation().getX();
      
        double yAngularOffset = Math.asin((targetPosition.get().getZ() - currentRelativePosition.get().getZ()) / getPlanarDistance(new Translation2d(currentRelativePosition.get().getX(), currentRelativePosition.get().getY()), new Translation2d(targetPosition.get().getX(), targetPosition.get().getY())).get());

        if (Double.isNaN(xAngularOffset) || Double.isNaN(yAngularOffset)) {
          return Optional.empty();
        }

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
     * or empty if no target is detected
     */
    public Optional<Double> getHorizontalVelocity(Translation2d targetPosition) {
      if (targetPos().isEmpty() || getHeightDifferenceToTarget().isEmpty()) {
        return Optional.empty();
      }

      return getPlanarDistance(targetPos().get().getTranslation().toTranslation2d(), targetPosition);
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