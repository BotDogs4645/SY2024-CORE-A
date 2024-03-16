package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class NodeStorage {

  public NodeStorage(Swerve swerveDrive, Field2d playingField) {
    nodes = new Node[] {
      // initalizeNode(0, new SequentialCommandGroup(
      //   new AdvanceToTarget(swerveDrive, playingField, true, new Transform2d(new Translation2d(13, 5.547868), new Rotation2d(0, 0))),
      //   launchNoteCommand
      // ),
      // new Translation2d(15.579342, 5.547868), 100)
    };
    // nodes = new Node[] {
    //     initalizeTargetNode(0, 
    //     new Translation2d(15.579342, 5.547868), 
    //     new Translation3d(), 
    //     3),
    // };
  }

  public static class Node {
    public int nodeID;
    public Command command;
    public Translation2d position;
    public double radius;
    public Translation3d targetPosition;
  }

  static Node initalizeNode(int nodeID, Command nodeCommand, Translation2d nodePosition, double nodeRadius) {
    Node nodeInstance = new Node();

    nodeInstance.nodeID = nodeID;
    nodeInstance.command = nodeCommand;
    nodeInstance.position = nodePosition;
    nodeInstance.radius = nodeRadius;

    return nodeInstance;
  }

  static Node initalizeTargetNode(int nodeID, Translation2d nodePosition, Translation3d targetPosition, double nodeRadius) {
    Node nodeInstance = new Node();

    nodeInstance.nodeID = nodeID;
    nodeInstance.position = nodePosition;
    nodeInstance.radius = nodeRadius;
    nodeInstance.targetPosition = targetPosition;

    return nodeInstance;
  }

  public Node[] nodes;
}