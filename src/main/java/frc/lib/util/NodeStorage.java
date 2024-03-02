package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AdvanceToTarget;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;

public class NodeStorage {

  private Swerve swerveDrive;
  private AprilTag aprilTagInstance;
  private Launcher launcherInstance;
  private IntakeIndexer intakeIndexerInstance;

  public NodeStorage(Swerve swerveDrive, AprilTag aprilTagInstance, Launcher launcherInstance, IntakeIndexer intakeIndexerInstance) {
    this.swerveDrive = swerveDrive;
    this.aprilTagInstance = aprilTagInstance;
    this.launcherInstance = launcherInstance;
    this.intakeIndexerInstance = intakeIndexerInstance;
  }
    public static class Node {
      public int nodeID;
      public SequentialCommandGroup command;
      public Translation2d position;
      public double radius;
    }

    static Node initalizeNode(int nodeID, SequentialCommandGroup nodeCommand, Translation2d nodePosition, double nodeRadius) {
      Node nodeInstance = new Node();

      nodeInstance.nodeID = nodeID;
      nodeInstance.command = nodeCommand;
      nodeInstance.position = nodePosition;
      nodeInstance.radius = nodeRadius;

      return nodeInstance;
    }

    public final Node[] nodes = {
      initalizeNode(0, new SequentialCommandGroup(
        Commands.run(() -> new AdvanceToTarget(swerveDrive, aprilTagInstance, true, new Transform2d(new Translation2d(15.579342, 5.547868), new Rotation2d(0, 0))), swerveDrive),
        Commands.run(() -> new Shoot(swerveDrive, launcherInstance, intakeIndexerInstance, aprilTagInstance, 0))
        ),
        new Translation2d(15.579342, 5.547868), 1)
    };
}
