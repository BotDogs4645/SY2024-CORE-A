// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.Pair;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/** Add your docs here. */
public class AprilTag {

    private class TargetVector {
        
    }

    private double tx, ty;
    private double angle;

    public AprilTag(double tx, double ty) {
      this.tx = tx;
      this.ty = ty;
    }

    public double[] targetToXYZVector() {
        double[] targetVector = new double[3];

        double d = getDirectDistance();

        double x = d * Math.sin(LimelightHelpers.getTX(""));
        targetVector[0] = x;

        return targetVector;
    }

    public double getDirectDistance() {
      return (/*placeholder*/3);
    }
}
