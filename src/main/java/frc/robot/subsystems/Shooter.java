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

public class Shooter extends SubsystemBase {
  /** Creates a new Launcher. */

  private TalonFX topMotor;
  private TalonFX bottomMotor;
  private CANcoder absEncoder;
  private CANSparkMax positionMotor;

  private PIDController controller;

  private double theta = 0;

  public Shooter() {
    absEncoder = new CANcoder(Constants.Launcher.absEncoderId, "*");

    topMotor = new TalonFX(Constants.Launcher.topMotorID, "*");

    bottomMotor = new TalonFX(Constants.Launcher.bottomMotorID, "*");

    positionMotor = new CANSparkMax(Constants.Launcher.angleMotorID, MotorType.kBrushless);

    controller = new PIDController(
      Constants.Launcher.PID.kP,
      Constants.Launcher.PID.kI,
      Constants.Launcher.PID.kD
    );

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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter angle power", positionMotor.get());
    SmartDashboard.putNumber("Shooter abs Encoder", absEncoder.getAbsolutePosition().getValue());
    // positionMotor.set(controller.calculate(
    //   absEncoder.getAbsolutePosition().getValue(), theta
    // ));
  }
}
