// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeIndexer extends SubsystemBase {
  /** Creates a new IntakeIndexer. */

  private PWMSparkMax feederMotor;
  private PWMSparkMax intakeMotor;

  private DigitalInput noteDetectionSwitch;

  public IntakeIndexer() {
    feederMotor = new PWMSparkMax(Constants.Intake.feederMotorPWMPort);
    intakeMotor = new PWMSparkMax(Constants.Intake.intakeMotorPWMPort);
    noteDetectionSwitch = new DigitalInput(Constants.Intake.noteDetectionSwitchDIOPort);
  }

  public void runFeeder(double speed) {
    feederMotor.set(speed);
  }

  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void stopFeeder() {
    feederMotor.set(0);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void toggleIntake() {
    if(intakeMotor.get() != 0) {
      intakeMotor.set(0);
    } else {
      intakeMotor.set(1);
    }
  }

  public void toggleFeeder() {
    if(feederMotor.get() != 0) {
      feederMotor.set(0);
    } else {
      feederMotor.set(-1);
    }
  }
  public void toggleBoth() {
    toggleIntake();
    toggleFeeder();
  }

  public boolean hasNote() {
    return noteDetectionSwitch.get();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
