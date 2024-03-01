package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.AprilTag;
import frc.robot.RobotContainer;
import frc.lib.util.NodeStorage;

public class NodalTaskExecution extends Command {

    public RobotContainer robotContainerInstance;
    public AprilTag aprilTagInstance;
    public AdvanceToTarget advanceToTargetInstance;

    public Optional<NodeStorage.Node> currentNode = Optional.empty();

    public NodalTaskExecution(RobotContainer robotContainerInstance, AprilTag aprilTagInstance, AdvanceToTarget advanceToTargetInstance) {
        this.robotContainerInstance = robotContainerInstance;
        this.aprilTagInstance = aprilTagInstance;
        this.advanceToTargetInstance = advanceToTargetInstance;
    }
    
    public Optional<NodeStorage.Node> detectCurrentNode(Pose2d currentPosition) {
        for (NodeStorage.Node currentNodePosition : NodeStorage.nodes) {
            if (aprilTagInstance.getPlanarDistance(currentPosition.getTranslation(), new Translation2d(currentNodePosition.position.getX(), currentNodePosition.position.getY())).get() < currentNodePosition.radius) {
                return Optional.of(currentNodePosition);
            }
        }

        return Optional.empty();
    }

    @Override
    public void initialize() {
        currentNode = detectCurrentNode(robotContainerInstance.getField().getRobotPose());

        if (currentNode.isPresent()) {

            advanceToTargetInstance.specifyTarget(new Transform2d(new Translation2d(currentNode.get().position.getX(), currentNode.get().position.getY()), new Rotation2d(0, 0)));
            advanceToTargetInstance.schedule();

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
