package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class NodeStorage {
    
    public class Node {
      int nodeID;
      Command command;
      Translation2d position;
      int radius;
    }

    public static final Node[] nodes = {};
}
