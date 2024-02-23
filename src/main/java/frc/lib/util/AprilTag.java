// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class AprilTag {

  private Translation2d vecToTarget;
  private double relX, relY, relZ;
  private Rotation2d rot;

  public AprilTag() {}

  public AprilTag(double x, double y, double z, Rotation2d rot) {
    relX = x;
    relY = y;
    relZ = z;
    this.rot = rot;
  }

  public void setX(double x) {
    relX = x;
  }

  public void setY(double y) {
    relY = y;
  }

  public void setZ(double z) {
    relZ = z;
  }
  
  public Translation2d toRobotRealativeTranslation() {
    return new Translation2d(relX, relY);
  }


}
