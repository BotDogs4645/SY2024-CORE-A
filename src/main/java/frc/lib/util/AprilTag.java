// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.beans.Visibility;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants.Vision;

/** Add your docs here. */
public class AprilTag {

  public static double getDirectDistance() {
    Transform3d targetVector = Limelight.targetPos();

    if (Limelight.targetPos() != null) {
      return Math.sqrt(Math.pow(targetVector.getX(), 2) + Math.pow(targetVector.getY(), 2) + Math.pow(targetVector.getZ(), 2));
    } else {
      return -1;
    }
  }

  public static Transform3d getTargetVector() {
    return Limelight.targetPos();
  }

}
