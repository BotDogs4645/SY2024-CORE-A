// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.beans.Visibility;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.Pair;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Vision;

/** Add your docs here. */
public class AprilTag {

  private class TargetVector {

  }

  private double tx, ty, height;
  private double angle;

  public AprilTag(double tx, double ty, double height) {
    this.tx = tx;
    this.ty = ty;
    this.height = height;
  }

  // public double[] targetToXYZVector() {
  // double[] targetVector = new double[3];

  // double d = getDirectDistance();

  // double x = d * Math.sin(LimelightHelpers.getTX(""));
  // targetVector[0] = x;

  // return targetVector;
  // }

  // public double getDirectDistance() {
  // return (/*placeholder*/3);
  // }

  public double[] targetToXYZVector() {
    double[] targetVector = new double[3];

    double y = height - Vision.kLimelightHeightMeters;
    double z = y / Math.tan(Math.toRadians(LimelightHelpers.getTY("") + Vision.kLimelightAngleDegrees)); // also need Math.abs()?
    double x = z * Math.tan(Math.toRadians(LimelightHelpers.getTX("")));

    targetVector[0] = x;
    targetVector[1] = y;
    targetVector[2] = z;

    return targetVector;
  }

  public double getDirectDistance() {
    double[] targetVector = targetToXYZVector();
    return (Math.pow(targetVector[0],2) + Math.pow(targetVector[1],2) + Math.pow(targetVector[2],2));
  }
}