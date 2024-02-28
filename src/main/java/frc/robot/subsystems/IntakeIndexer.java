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
 
  private boolean hasNote;

  public IntakeIndexer() {
    feederMotor = new PWMSparkMax(Constants.Intake.feederMotorPWMPort);
    intakeMotor = new PWMSparkMax(Constants.Intake.intakeMotorPWMPort);
    feederMotor.setInverted(Constants.Intake.invertFeederMotor);
    intakeMotor.setInverted(Constants.Intake.invertIntakeMotor);
    noteDetectionSwitch = new DigitalInput(Constants.Intake.noteDetectionSwitchDIOPort);
  }

  public void setHasNote(boolean hasNote) {
    this.hasNote = hasNote;
  }

  public boolean hasNote() {
    return this.hasNote;
  }

  public void run(double speed) {
    feederMotor.set(speed);
    intakeMotor.set(speed);
  }

  public void run() {
    run(Constants.Intake.intakeSpeed);
  }

  public void stop() {
    feederMotor.set(0);
    intakeMotor.set(0);
  }

  public void toggle() {
    if(intakeMotor.get() != 0) {
      stop();
    } else {
      run(Constants.Intake.intakeSpeed);
    }
  }

  public void startSpittingNote() {
    run(-1);
    this.hasNote = false;
  }

  public boolean getLimitSwitch() {
    //Limit switch is pulled up by default, and low when activated
    return !noteDetectionSwitch.get();
  }

  public boolean isRunning() {
    return intakeMotor.get() != 0;
  };
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
