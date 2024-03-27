package frc.lib.util;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class NodeStorage {

  public NodeStorage(Swerve swerveDrive, Field2d playingField) {
    nodes = new Node[] {};
  }

  public static class Node {
    public int nodeID;
    public Command command;
    public Translation2d position;
    public double radius;
    public Translation3d targetPosition;
  }

  static Node initalizeNode(int nodeID, SequentialCommandGroup nodeCommand, Translation2d nodePosition, double nodeRadius) {
    Node nodeInstance = new Node();

    nodeInstance.nodeID = nodeID;
    nodeInstance.command = nodeCommand;
    nodeInstance.position = nodePosition;
    nodeInstance.radius = nodeRadius;

    return nodeInstance;
  }

  public Node[] nodes;
}