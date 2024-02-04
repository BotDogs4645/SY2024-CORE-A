package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.AprilTag;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/**
 * The Limelight subsystem.
 * 
 * Limelight documentation: https://limelightvision.io/
 * 
 * We use the Limelight for location estimation with AprilTags.
 */
public class Limelight extends SubsystemBase {

    private AprilTag currentTag = new AprilTag(0, 0, 0);
 
    public boolean hasTarget() {
        return (LimelightHelpers.getTA("") < 1E-6) ? true : false;
    }

    public Pose3d getTargetPose() {
        return LimelightHelpers.getTargetPose3d_RobotSpace("");
    }

    public double getTx() {
        return LimelightHelpers.getTX("");
    }

    public double getTy() {
        return LimelightHelpers.getTY("");
    }

    public double getTargetHeight() {
        Transform3d tag = Constants.APRILTAGS.get(getTargetID());
        if(tag != null) {
            return tag.getZ();
        } else {
            return 0;
        }
    }

    public AprilTag getAprilTag() {
        return currentTag;
    }

    public int getTargetID() {
        return (int) LimelightHelpers.getFiducialID("");
    }

    public Pose3d test() {
        return LimelightHelpers.getTargetPose3d_RobotSpace("");
    }


    @Override
    public void periodic() {
      currentTag = new AprilTag(
        getTx(),
        getTy(),
        getTargetHeight()
      );
    }
    
}
