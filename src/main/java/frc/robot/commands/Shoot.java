// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.lib.util.AprilTag;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeIndexer;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.lib.util.NodeStorage.Node;
import java.util.Optional;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  Swerve swerveInstance;
  Launcher launcher;
  IntakeIndexer intakeIndexer;
  AprilTag aprilTagInstance;
  NodalTaskExecution nodalTaskExecutionInstance;
  Optional<Node> currentNode;
  
  public Shoot(Swerve swerveInstance, Launcher launcher, IntakeIndexer intakeIndexer, AprilTag aprilTagInstance, NodalTaskExecution nodalTaskExecutionInstance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveInstance = swerveInstance;
    this.launcher = launcher;
    this.intakeIndexer = intakeIndexer;
    this.aprilTagInstance = aprilTagInstance;
    this.nodalTaskExecutionInstance = nodalTaskExecutionInstance;

    addRequirements(launcher, intakeIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentNode = nodalTaskExecutionInstance.detectCurrentNode(swerveInstance.getPose());
    if (currentNode.isPresent()) {
        launcher.aimLauncher(currentNode.get().nodeID);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int currentNodeID = currentNode.get().nodeID;

    if (currentNodeID == Constants.Launcher.ampNodeID){
      vroom(Constants.Launcher.ampSpeed);
    }
    else if (currentNodeID == Constants.Launcher.speakerNodeID){
      vroom(Constants.Launcher.speakerSpeed);
    }
    else if (currentNodeID == Constants.Launcher.trapNodeID){
      vroom(Constants.Launcher.trapSpeed);
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.stopLauncher();
    intakeIndexer.stopIntake();
    intakeIndexer.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public void vroom(double speed){
    intakeIndexer.runIntake(speed);
    intakeIndexer.runFeeder(speed);
    launcher.startLauncher(speed);
  }
}