// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import com.ctre.phoenix6.signals.*;

public class Arm extends ProfiledPIDSubsystem {
  private ArmFeedforward arm;
  private double pidWant;
  private double ffWant;
  private TalonFX leftMotor;
  private TalonFX rightMotor;
  private CANcoder cancoder;
  public double wantedAngle;
  private ShuffleboardTab tab;
      StatusSignal<Double> absolutePosition;
      StatusSignal<Double> velocity;
      StatusSignal<Double> leftMotorAmp;
      StatusSignal<Double> rightMotorAmp;
      StatusSignal<Double> leftMotorVolts;
      StatusSignal<Double> rightMotorVolts;
      StatusSignal<Double> rotations;
      double degrees;

public static class ArmControlRequest {
    private Pose2d robotPosition;
    private TrapezoidProfile.State pendulumRotationAngle;

    // simple trig to determine pendulum rotation angle and bot position
    public ArmControlRequest(Pose3d endEffector) {
      // The equation below graphs a semi-circle:
      // If the arm length is not long enough to reach the desired HEIGHT, then it
      // will return a translation with a NaN X component
      double distanceXFromEndEffector = Math.sqrt(
          Math.pow(ArmConstants.armLength, 2) - Math.pow(endEffector.getZ() - ArmConstants.heightOfAxis, 2));

      robotPosition = new Pose2d(
          new Translation2d(endEffector.getX() + distanceXFromEndEffector, endEffector.getY()),
          Rotation2d.fromRadians(0));

      pendulumRotationAngle = new TrapezoidProfile.State(
          Math.asin((-ArmConstants.heightOfAxis + endEffector.getZ()) / ArmConstants.armLength) + Math.toRadians(9), // gear ratio offset
          0);
    }

    public Pose2d getRobotPosition() {
      return robotPosition;
    }

    public TrapezoidProfile.State getArmRotation() {
      return pendulumRotationAngle;
    }
  }
  

  /** Creates a new PendulumProfiled. */
  public Arm() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            8.25,
            0,
            0.05,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1.5, 0.7)));

    this.rightMotor = new TalonFX(15);
    this.leftMotor = new TalonFX(14);
    this.cancoder = new CANcoder(16);

    this.arm = new ArmFeedforward(0.48005, 0.54473, 1.3389, 0.19963);
    this.ffWant = 0;

    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setInverted(true);

    super.getController().setTolerance(1.5, .5);

    this.tab = Shuffleboard.getTab("Arm");
    absolutePosition = cancoder.getAbsolutePosition();
    tab.addNumber("Arm Absolute Angle (degrees)", () -> absolutePosition.getValue());
    velocity = cancoder.getVelocity();
    tab.addNumber("Arm velo (degrees / second)", () ->  velocity.getValue());
    leftMotorAmp = leftMotor.getStatorCurrent();
    tab.addNumber("Arm left amp", () -> leftMotorAmp.getValue());
    rightMotorAmp = rightMotor.getStatorCurrent();
    tab.addNumber("Arm right amp", () -> rightMotorAmp.getValue());
    leftMotorVolts = leftMotor.getMotorVoltage();
    tab.addNumber("Arm left volts", () -> leftMotorVolts.getValue());
    rightMotorVolts = rightMotor.getMotorVoltage();
    tab.addNumber("Arm right volts", () -> rightMotorVolts.getValue());

    tab.addNumber("PID Want", () -> getPIDVoltage());
    tab.addNumber("FF want", () -> getFFVoltage());


    tab.add(this);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    ffWant = arm.calculate(setpoint.position, setpoint.velocity);

    leftMotor.setVoltage(output + ffWant);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getArmPosition();
  }

  public double getArmPosition() {
    return  absolutePosition.getValue() * (Math.PI / 180.0);
  }

  public double getArmVelocity() {
    return  velocity.getValue() * (Math.PI / 180.0);
  }

  public double getError() {
    return super.getController().getPositionError();
  }

  public boolean atSetpoint() {
    return super.getController().atSetpoint();
  }

  public double getPIDVoltage() {
    return pidWant;
  }

  public void setWantedAngle(double wantedAngle) {
    this.wantedAngle = wantedAngle;
  }


  public double getFFVoltage() {
    return ffWant;
  }
public void moveArm(int id) {
        rotations = cancoder.getPosition();
        degrees = rotations.getValue() * 360.0;
        if (id == 1 && degrees < -45) {
            leftMotor.set(0.4);
            rightMotor.set(0.4);

        }else if (id == 2 && degrees < 0) {
            leftMotor.set(0.4);
            rightMotor.set(0.4);
        } else {
            leftMotor.set(0);
            rightMotor.set(0);
            leftMotor.stopMotor();
            rightMotor.stopMotor();
         }

    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
    }
  }

