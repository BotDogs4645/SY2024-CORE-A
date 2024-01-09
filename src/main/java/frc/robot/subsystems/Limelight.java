package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Limelight subsystem.
 * 
 * Limelight documentation: https://limelightvision.io/
 * 
 * We use the Limelight for location estimation with AprilTags.
 */
public class Limelight extends SubsystemBase {

    /**
     * Uses the Limelight to retrieve target values.
     * This will be null if there is no target detected.
     * See {@link #targetId()} for notes on using the ID stored within the pair.
     * @return a pair that includes the target's ID and transformation relative
     *         to the robot's center position.
     */
    public Pair<Integer, Transform3d> target() {
        return null; // TODO
    }

    /**
     * Uses the Limelight to retrieve the ID of the target.
     * If no target is spotted, this returns -1.
     * If multiple targets, including the desired one, are visible, the
     * Limelight will use the most confident result for position estimation.
     * We therefore recommend that you do not use this ID and instead use the
     * desired location along with {@link #targetPos()}.
     * @return the ID of the target spotted, or -1 if there is not one.
     */
    public int targetId() {
        var target = target();
        return target != null ? target.getFirst() : -1;
    }

    /**
     * Uses the Limelight to retrieve the offset of the target, relative to the
     * robot's center position.
     * If no target is spotted, this returns null.
     * @return the spotted target offset, or null if none
     */
    public Transform3d targetPos() {
        var target = target();
        return target != null ? target.getSecond() : null;
    }

    /**
     * Estimates the position of the robot, using the Limelight's knowledge of
     * the position of each AprilTag in the game board.
     * If no AprilTags are spotted, this returns null.
     * 
     * For a map of the game board and AprilTag positions, see
     * {@link https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf},
     * page 4.
     * 
     * @return the robot's estimated position in the board.
     */
    public Transform3d estimatedPosition() {
        return null; // TODO
    }


    
}
