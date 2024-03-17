// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeIndexer extends SubsystemBase {
  /** Creates a new IntakeIndexer. */

  private PWMSparkMax feederMotor;
  private PWMSparkMax intakeMotor;

  private DigitalInput noteDetectionSwitch;
  private DigitalInput photogate;
 
  private boolean hasNote;
  private double initIntakeTime = 0;
  private double tripTime = 0;

  public IntakeIndexer() {
    feederMotor = new PWMSparkMax(Constants.Intake.feederMotorPWMPort);
    intakeMotor = new PWMSparkMax(Constants.Intake.intakeMotorPWMPort);
    feederMotor.setInverted(Constants.Intake.invertFeederMotor);
    intakeMotor.setInverted(Constants.Intake.invertIntakeMotor);
    noteDetectionSwitch = new DigitalInput(Constants.Intake.noteDetectionSwitchDIOPort);
    photogate = new DigitalInput(2);
  }

  public void setHasNote(boolean hasNote) {
    this.hasNote = hasNote;
  }

  public boolean hasNote() {
    return this.hasNote;
  }

  public void setSpeed(double speed) {
    feederMotor.set(speed);
    intakeMotor.set(speed);
  }

  public boolean getPhotogate() {
    return photogate.get();
  }


  public void stop() {
    feederMotor.set(0);
    intakeMotor.set(0);
  }

  public double getSpeed() {
    return feederMotor.get();
  }

  public void intakeFromSource() {
    setSpeed(-0.35);
    System.out.println("AHHH");
  }

  public void toggle() {
    if(intakeMotor.get() != 0) {
      stop();
    } else {
      setSpeed(Constants.Intake.intakeSpeed);
    }
  }

  public void startSpittingNote() {
    setSpeed(-1);
    this.hasNote = false;
  }

  public boolean getLimitSwitch() {
    //Limit switch is pulled up by default, and low when activated
    return !noteDetectionSwitch.get();
  }

  public boolean isRunning() {
    return intakeMotor.get() != 0;
  };

  public double getTripTime() {
    return tripTime;
  }

  public void resetSwitch() {
    initIntakeTime = 0;
  }
  
  @Override
  public void periodic() {
    
    
    if(getLimitSwitch() == true) {
      hasNote = true;
      initIntakeTime = Timer.getFPGATimestamp();
    }
    if (getLimitSwitch() == true && hasNote) {
      tripTime = Timer.getFPGATimestamp() - initIntakeTime;
    }


    SmartDashboard.putNumber("curr Time", tripTime);
    SmartDashboard.putNumber("init time", initIntakeTime);
  }


}