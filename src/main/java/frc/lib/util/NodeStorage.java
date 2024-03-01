package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class NodeStorage {
    
    public static class Node {
      public int nodeID;
      public SequentialCommandGroup command;
      public Translation2d position;
      public int radius;
    }

    static Node initalizeNode(int nodeID, SequentialCommandGroup nodeCommand, Translation2d nodePosition, int nodeRadius) {
      Node nodeInstance = new Node();

      nodeInstance.nodeID = nodeID;
      nodeInstance.command = nodeCommand;
      nodeInstance.position = nodePosition;
      nodeInstance.radius = nodeRadius;

      return nodeInstance;
    }

    public static final Node[] nodes = {
      initalizeNode(0, new SequentialCommandGroup(), new Translation2d(), 0)
    };
}
