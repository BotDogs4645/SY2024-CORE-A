package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Limelight subsystem.
 * 
 * Limelight documentation: https://limelightvision.io/
 * 
 * We use the Limelight for location estimation with AprilTags.
 */
public class Limelight extends SubsystemBase {

    public static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("limelight");

    /**
    * Fetches and @return(s) the NetworkTable entry of 
    * the parameter provided in the method call.
    */
    public Optional<NetworkTableEntry> entry(String key) {
        return Optional.of(TABLE.getEntry(key));
    }

    /** 
    * Uses the Limelight to fetch, calculate, and @return the offset 
    * from the robot to the current primary apriltag target.
    */
    public Optional<Transform3d> targetPos() {
    Optional<double[]> targetPose = Optional.of(entry("targetpose_robotspace").get().getDoubleArray(new double[0]));
    

    if (targetPose.isPresent()) {
        return Optional.of(new Transform3d(
            new Translation3d(targetPose.get()[0], targetPose.get()[1], targetPose.get()[2]),
            new Rotation3d(targetPose.get()[3], targetPose.get()[4], targetPose.get()[5])
        ));
    } else {
        return Optional.empty();
    }
    };

    /**
     * Uses the Limelight to retrieve the ID of the target.
     * If no target is spotted, this returns "Optional.empty()."
     * If multiple targets, including the desired one, are visible, the
     * Limelight will use the most confident result for position estimation.
     * We therefore recommend that you do not use this ID and instead use the
     * desired location along with {@link #targetPos()}.
     * 
     * @return the ID of the target spotted, or 'Optional.empty()' 
     * if there is not one.
     */
    public Optional<Integer> targetId() {
        int aprilTagID = (int) TABLE.getEntry("tid").getInteger(-1);
        if (aprilTagID == -1) return Optional.empty();

        return Optional.of(aprilTagID);
    }
}
