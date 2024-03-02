package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AprilTag;
import frc.lib.util.NodeStorage;

public class NodalTaskExecution extends Command {

    public AprilTag aprilTagInstance;
    public Field2d playingField;
    public NodeStorage nodeStorageInstance;

    public Optional<NodeStorage.Node> currentNode = Optional.empty();

    public NodalTaskExecution(AprilTag aprilTagInstance, Field2d playingField, NodeStorage nodeStorageInstance) {
        this.aprilTagInstance = aprilTagInstance;
        this.playingField = playingField;
        this.nodeStorageInstance = nodeStorageInstance;
    }
    
    public Optional<NodeStorage.Node> detectCurrentNode(Pose2d currentPosition) {
        for (NodeStorage.Node currentNode : nodeStorageInstance.nodes) {
            if (aprilTagInstance.getPlanarDistance(currentPosition.getTranslation(), new Translation2d(currentNode.position.getX(), currentNode.position.getY())).get() < currentNode.radius) {
                return Optional.of(currentNode);
            }
        }

        return Optional.empty();
    }

    @Override
    public void initialize() {
        currentNode = detectCurrentNode(playingField.getRobotPose());

        if (currentNode.isPresent()) {
            currentNode.get().command.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        currentNode.get().command.cancel();
        currentNode = Optional.empty();
    }

    @Override 
    public boolean isFinished() {
        return currentNode.isEmpty() || currentNode.get().command.isFinished();
    }
}
