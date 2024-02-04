// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.beans.Visibility;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Vision;

/** Add your docs here. */
public class AprilTag {


  private double tx, ty, height;
  private Pose3d targetPose;

  public AprilTag(double tx, double ty, double height) {
    this.tx = tx;
    this.ty = ty;
    this.height = height;

    this.targetPose = toPose3d(targetToXYZVector(), tx);
  }

  private Pose3d toPose3d(double[] vec, double deltaTheta) {
    return new Pose3d(
      vec[0],
      vec[1],
      vec[2],
      new Rotation3d(0, 0, deltaTheta)
    );
  }

  public double[] targetToXYZVector() {
    double[] targetVector = new double[3];

    double z = height - Vision.kLimelightHeightMeters;
    double y = z / Math.tan(Math.toRadians(ty + Vision.kLimelightAngleDegrees)); // also need Math.abs()?
    double x = y * Math.tan(Math.toRadians(tx));

    targetVector[0] = x;
    targetVector[1] = y;
    targetVector[2] = z;

    return targetVector;
  }

  public Pose3d getTagPose() {
    return targetPose;
  }

  public Translation2d getTranslationToTag() {
    return new Translation2d(targetToXYZVector()[0], targetToXYZVector()[1]);
  }
  public Rotation2d getDeltaRoationToTag() {
    return new Rotation2d(targetPose.getRotation().getZ());
  }

  public double getDirectDistance() {
    double[] targetVector = targetToXYZVector();
    return (Math.pow(targetVector[0], 2) + Math.pow(targetVector[1], 2) + Math.pow(targetVector[2], 2));
  }
  

}
