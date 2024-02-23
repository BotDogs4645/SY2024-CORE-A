package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase{
  private TalonFX bottomLaunchMotor, topLaunchMotor;
  private CANSparkMax aimLaunchMotor;
  private CANcoder cancoder;
  private SparkPIDController controller;
  private double wantedAngle;
  private Limelight limelight;

 public Launcher(Limelight limelight) {
    topLaunchMotor = new TalonFX(Constants.Launcher.topMotorID);
    bottomLaunchMotor = new TalonFX(Constants.Launcher.bottomMotorID);
    aimLaunchMotor = new CANSparkMax(Constants.Launcher.angleMotorID, MotorType.kBrushless);
    controller = aimLaunchMotor.getPIDController();
    controller.setPositionPIDWrappingEnabled(false);
    controller.setPositionPIDWrappingMinInput(0);
    controller.setPositionPIDWrappingMaxInput(90);
    

  
    bottomLaunchMotor.setInverted(true);
    cancoder = new CANcoder(0);
  }
  public void startLauncher(double speed) {
    bottomLaunchMotor.set(speed);
    topLaunchMotor.set(speed);
  }

  public void stopLauncher() {
    topLaunchMotor.set(0);
    bottomLaunchMotor.set(0);
    aimLaunchMotor.set(0);
  }

  public double getMeasurement() {
    // Return the process variable measurement here
    return getAimPosition();
  }

  public double getAimPosition() {
    // Math to get the degrees of the aimLaunchMotor
    return cancoder.getPosition().getValue() * (Math.PI / 180.0);
  }

  public void setWantedAngle(double wantedAngle) {
    this.wantedAngle = wantedAngle;
  }
  public void aimLauncher() {
    //sets the setpoint angle to the angle calculated by the getLaunchAngle method
    wantedAngle = limelight.getLaunchAngle().getAsDouble();
    controller.setReference(wantedAngle, CANSparkMax.ControlType.kPosition);
   
  }
}
