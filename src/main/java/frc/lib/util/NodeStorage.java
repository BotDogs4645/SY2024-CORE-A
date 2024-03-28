package frc.lib.util;

import frc.robot.Constants;
import frc.robot.commands.CommandBuilder;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        CommandBuilder.AdvanceToTarget(swerveDrive, playingField, new Pose2d(1.8415, 8.2042 - Constants.AdvanceToTarget.halfRobotLength, new Rotation2d(-90, 0))), 
        CommandBuilder.ShootAmp(intakeIndexer, shooter, pneumatics)
      ),
      new Translation2d(1.8415, 8.2042 - Constants.AdvanceToTarget.halfRobotLength),
      0.75
      ),
      initalizeNode(
        1, 
        new SequentialCommandGroup(
          CommandBuilder.AdvanceToTarget(swerveDrive, playingField, new Pose2d(-0.0381 + 0.917575 + Constants.AdvanceToTarget.halfRobotLength, 5.547868, new Rotation2d(-180, 0))),
          CommandBuilder.ShootSpeaker(intakeIndexer, shooter)
        ), 
        new Translation2d(-0.0381 + 0.917575 + Constants.AdvanceToTarget.halfRobotLength, 5.547868), 
        0.75
      ),
      initalizeNode(
        2, 
        new SequentialCommandGroup(
          CommandBuilder.AdvanceToTarget(swerveDrive, playingField, new Pose2d(15.079472 - Constants.AdvanceToTarget.halfRobotLength, 0.245872 + Constants.AdvanceToTarget.halfRobotLength, new Rotation2d(60, 0))),
          CommandBuilder.IntakeFromSource(intakeIndexer, shooter)
        ), 
        new Translation2d(15.079472 - Constants.AdvanceToTarget.halfRobotLength, 0.245872 + Constants.AdvanceToTarget.halfRobotLength),
        0.75
      )
    };

    redNodes = new Node[] {
      initalizeNode(
      3, 
      new SequentialCommandGroup(
        CommandBuilder.AdvanceToTarget(swerveDrive, playingField, new Pose2d(14.700758, 8.2042 - Constants.AdvanceToTarget.halfRobotLength, new Rotation2d(-90, 0))), 
        CommandBuilder.ShootAmp(intakeIndexer, shooter, pneumatics)
      ),
      new Translation2d(14.700758, 8.2042 - Constants.AdvanceToTarget.halfRobotLength), 
      0.75
      ),
      initalizeNode(
        4, 
        new SequentialCommandGroup(
          CommandBuilder.AdvanceToTarget(swerveDrive, playingField, new Pose2d(16.579342 - 0.917575 - Constants.AdvanceToTarget.halfRobotLength, 5.547868, new Rotation2d(0, 0))),
          CommandBuilder.ShootSpeaker(intakeIndexer, shooter)
        ), 
        new Translation2d(16.579342 - 0.917575 - Constants.AdvanceToTarget.halfRobotLength, 5.547868), 
        0.75
      ),
      initalizeNode(
        5, 
        new SequentialCommandGroup(
          CommandBuilder.AdvanceToTarget(swerveDrive, playingField, new Pose2d(0.356108 + Constants.AdvanceToTarget.halfRobotLength, 0.883666 + Constants.AdvanceToTarget.halfRobotLength, new Rotation2d(120, 0))),
          CommandBuilder.IntakeFromSource(intakeIndexer, shooter)
        ), 
        new Translation2d(0.356108 + Constants.AdvanceToTarget.halfRobotLength, 0.883666 + Constants.AdvanceToTarget.halfRobotLength), 
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