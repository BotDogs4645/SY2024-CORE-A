// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  /** Creates a new Launcher. */

  private TalonFX topMotor;
  private TalonFX bottomMotor;
  private CANSparkMax positionMotor;
  private boolean mode;

  public Shooter() {
    topMotor = new TalonFX(Constants.Launcher.topMotorID, "*");

    bottomMotor = new TalonFX(Constants.Launcher.bottomMotorID, "*");

    positionMotor = new CANSparkMax(Constants.Launcher.angleMotorID, MotorType.kBrushless);

    mode = false;

  }

  public void setShooterSpeed(double speed) {
    topMotor.set(speed);
    bottomMotor.set(-speed);
  }

  public void stop() {
    topMotor.set(0);
    bottomMotor.set(0);
  }

  public void toggleShooter() {
    if(topMotor.get() != 0) {
      topMotor.set(0);
      bottomMotor.set(0);
    } else {
      topMotor.set(0.65);
      bottomMotor.set(-0.65);
    }
  }

   public void toggleShooterAmp() {
    if(topMotor.get() != 0) {
      topMotor.set(0);
      bottomMotor.set(0);
    } else {
      topMotor.set(0.1475);
      bottomMotor.set(-0.1475);
    }
  }

  public void intakeFromSource() {
    topMotor.set(-0.125);
    bottomMotor.set(0.125);
  }

  public void setShooterAngle(double theta) {
    // this.theta = theta;
    positionMotor.set(theta);
  }
  public boolean getMode() {
    return mode;
  }

  @Override
  public void periodic() {
    mode = RobotContainer.manipulatorController.a().getAsBoolean();
    SmartDashboard.putNumber("shooter angle power", positionMotor.get());
  }
}
