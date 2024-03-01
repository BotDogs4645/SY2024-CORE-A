// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

public class IntakeIndexer extends SubsystemBase {
  /** Creates a new IntakeIndexer. */

  private PWMSparkMax feederMotor;
  private PWMSparkMax intakeMotor;

  public boolean intakeEnabled;
  public boolean feederEnabled;

  private DigitalInput noteDetectionSwitch;

  public IntakeIndexer() {
    feederMotor = new PWMSparkMax(Constants.Intake.feederMotorPWMPort);
    intakeMotor = new PWMSparkMax(Constants.Intake.intakeMotorPWMPort);
    noteDetectionSwitch = new DigitalInput(Constants.Intake.noteDetectionSwitchDIOPort);
  }

  public void runFeeder(double speed) {
    feederMotor.set(speed);

    if (speed != 0) {
      feederEnabled = true;
    } else {
      feederEnabled = false;
    }
  }

  public void runIntake(double speed) {
    intakeMotor.set(speed);

    if (speed != 0) {
      intakeEnabled = true;
    } else {
      intakeEnabled = false;
    }
  }

  public void stopFeeder() {
    feederMotor.set(0);
    feederEnabled = false;
  }

  public void stopIntake() {
    intakeMotor.set(0);
    intakeEnabled = false;
  }

  public BooleanSupplier hasNote() {
    BooleanSupplier noteDetected = () -> noteDetectionSwitch.get();

    return noteDetected;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}