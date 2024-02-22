package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    public static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("limelight");

    // Fetches and returns the NetworkTable entry of the parameter provided in the method call.
    public Optional<NetworkTableEntry> entry(String key) {
        return Optional.of(TABLE.getEntry(key));
    }

    // Fetches, calculates, and returns the offset from the robot to the current primary apriltag target.
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
}
