package frc.lib.util;

import frc.robot.commands.CommandBuilder;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class NodeStorage {

  public NodeStorage(Swerve swerveDrive, Field2d playingField, IntakeIndexer intakeIndexer, Shooter shooter, Pneumatics pneumatics) {
    blueNodes = new Node[] {
      initalizeNode(
      0, 
      new SequentialCommandGroup(
        CommandBuilder.AdvanceToTarget(swerveDrive, playingField, null), 
        CommandBuilder.ShootAmp(intakeIndexer, shooter, pneumatics)
      ),
      new Translation2d(), 
      0.75
      ),
      initalizeNode(
        1, 
        new SequentialCommandGroup(
          CommandBuilder.AdvanceToTarget(swerveDrive, playingField, null),
          CommandBuilder.ShootSpeaker(intakeIndexer, shooter)
        ), 
        new Translation2d(), 
        0.75
      ),
      initalizeNode(
        2, 
        new SequentialCommandGroup(
          CommandBuilder.AdvanceToTarget(swerveDrive, playingField, null),
          CommandBuilder.IntakeFromSource(intakeIndexer, shooter)
        ), 
        new Translation2d(), 
        0.75
      )
    };

    redNodes = new Node[] {
      initalizeNode(
      3, 
      new SequentialCommandGroup(
        CommandBuilder.AdvanceToTarget(swerveDrive, playingField, null), 
        CommandBuilder.ShootAmp(intakeIndexer, shooter, pneumatics)
      ),
      new Translation2d(), 
      0.75
      ),
      initalizeNode(
        4, 
        new SequentialCommandGroup(
          CommandBuilder.AdvanceToTarget(swerveDrive, playingField, null),
          CommandBuilder.ShootSpeaker(intakeIndexer, shooter)
        ), 
        new Translation2d(), 
        0.75
      ),
      initalizeNode(
        5, 
        new SequentialCommandGroup(
          CommandBuilder.AdvanceToTarget(swerveDrive, playingField, null),
          CommandBuilder.IntakeFromSource(intakeIndexer, shooter)
        ), 
        new Translation2d(), 
        0.75
      )
    };
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

  public Node[] blueNodes;
  public Node[] redNodes;
}