package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.LimelightHelpers.LimelightResults;

public class Launcher extends SubsystemBase{
  private TalonFX bottomLaunchMotor, topLaunchMotor;
  private CANSparkMax aimLaunchMotor;
  private CANcoder cancoder;
  private SparkPIDController controller;
  private double wantedAngle;

 public Launcher() {
    topLaunchMotor = new TalonFX(13);
    bottomLaunchMotor = new TalonFX(14);
    aimLaunchMotor = new CANSparkMax(18, MotorType.kBrushless);
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
    // TODO: Was causing build errors. Re-implement this section.
    // wantedAngle = limelight.getLaunchAngle();
    controller.setReference(wantedAngle, CANSparkMax.ControlType.kPosition);
   
  }
}
