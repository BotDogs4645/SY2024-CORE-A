package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class contains constant values. Generally, when there are magic
 * numbers within your code (numbers that are not derived from a formula in the
 * code, like calibration values) they should go here. This encourages
 * cleanliness.
 */
public class Constants {


    public static final int kDriverControllerPort = 0;
    public static final int pcmCanID = 1;

    public static class Limelight {

        // Limelight offset relative to the center of the robot.
        // Measured in meters.
        // These values do not do anything in themselves; you will need to add
        // them to the Limelight's active pipeline. They're just here for
        // reference.
        public static final double OFFSET_RIGHT = 0.32;
        public static final double OFFSET_UP = 0.55;
        public static final double OFFSET_FORWARDS = 0.0;
    }
    public static class ArmConstants {
        public static enum PendulumCommand {
            IdleWithPiece(-Math.PI / 4),
            Idle(-Math.PI / 2),
            Straight(0),
            RetrieveAngle(Math.toRadians(-6)), //TODO: find retrieve angle
            ;

            private double angle;

            private PendulumCommand(double position) {
                this.angle = position;
            }

            public double get() {
                return angle;
            }
            
        }
        /* State Space Settings */
        public static double measurementDelay = 0.10339;

        // Characterization
        public static final double kS = 0.084424;
        public static final double kV = 0.67768;
        public static final double kA = 0.13784;

        // Other
        public static final double momentOfInertia = 5.2; // 17.43589
        public static final double gearing = 100.0 / 1.0; // output / input
        public static final TrapezoidProfile.Constraints pendulumConstraints = new TrapezoidProfile.Constraints(
                Units.degreesToRadians(45),
                Units.degreesToRadians(90)); // Max arm speed and acceleration.

        public static final double heightOfAxis = Units.inchesToMeters(50.847); // from Drew
        public static final double armLength = Units.inchesToMeters(45.011);
    }

    /**
     * A key-value map, mapping each AprilTag to its position on the 2024
     * Crescendo board.
     * 
     * To retrieve a value, you can do {@code APRILTAGS.get(number)} and it will
     * return the transform of the april tag, or null if the number does not
     * have an associated AprilTag.
     * 
     * For a map of the game board and AprilTag positions, see
     * {@link https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf},
     * page 4.
     */
    public static final Map<Integer, Transform3d> APRILTAGS = Map.ofEntries(
            tag(1, 593.68, 9.68, 53.38, 120),
            tag(2, 637.21, 34.79, 53.38, 120),
            tag(3, 652.73, 196.17, 57.13, 180),
            tag(4, 652.73, 218.42, 57.13, 180),
            tag(5, 578.77, 323.00, 53.38, 270),
            tag(6, 72.5, 323.00, 53.38, 270),
            tag(7, -1.50, 218.42, 57.13, 0),
            tag(8, -1.50, 196.17, 57.13, 0),
            tag(9, 14.02, 34.79, 53.38, 60),
            tag(10, 57.54, 9.68, 53.38, 60),
            tag(11, 468.69, 146.19, 52.00, 300),
            tag(12, 468.69, 177.10, 52.00, 60),
            tag(13, 441.74, 161.62, 52.00, 180),
            tag(14, 209.48, 161.62, 52.00, 0),
            tag(15, 182.73, 177.10, 52.00, 120),
            tag(16, 182.73, 146.19, 52.00, 240));

    private static Map.Entry<Integer, Transform3d> tag(int id, double x, double y, double z, double rot) {
        final double inchesPerMeter = 39.37;

        return Map.entry(id, new Transform3d(
                new Translation3d(x / inchesPerMeter, y / inchesPerMeter, z / inchesPerMeter),
                new Rotation3d(rot, 0, 0)));
    }

}
