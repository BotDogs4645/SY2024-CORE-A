// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;

/**
 * The robot container class. This houses the subsystems and a lot of the core
 * logic.
 */
public class RobotContainer {

  private final Limelight limelight = new Limelight();
  private final Arm arm = new Arm();
  private final Pneumatics m_pneumaticsSubsystem = new Pneumatics();
  private final CommandXboxController m_driverController = new CommandXboxController(Constants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.b().onTrue(m_pneumaticsSubsystem.toggleClimber());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Limelight getLimelight() {
    return limelight;
  }

  public Arm getArm() {
    return arm;
  }
}
