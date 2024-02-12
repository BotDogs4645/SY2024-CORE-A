package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.PIDController;
import com.revrobotics.SparkRelativeEncoder;

// import edu.wpi.first.apriltag.AprilTag;
import frc.lib.util.AprilTag;
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
import frc.lib.util.LaunchCalculations;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.LimelightHelpers.LimelightResults;

public class Launcher extends SubsystemBase{
  private CANSparkMax rightLaunchMotor, leftLaunchMotor;
  private TalonFX aimLaunchMotor;
  private CANcoder cancoder;
  // all the rotational PID stuff
  private ArmFeedforward feedforward;
  private double pidWant;
  private double ffWant;
  public double wantedAngle;
  private ShuffleboardTab tab;
  
 public Launcher() {

    this.feedforward = new ArmFeedforward(0.48005, 0.54473, 1.3389, 0.19963);
    this.ffWant = 0;
    leftLaunchMotor = new CANSparkMax(16, MotorType.kBrushless);
    rightLaunchMotor = new CANSparkMax(17, MotorType.kBrushless);
    aimLaunchMotor = new TalonFX(18);
    rightLaunchMotor.setInverted(true);
    cancoder = new CANcoder(19);
    aimLaunchMotor.setNeutralMode(NeutralModeValue.Brake);



    this.tab = Shuffleboard.getTab("Arm");
    tab.addNumber("Aim Absolute Angle (degrees)", () -> (cancoder.getAbsolutePosition()).getValue());
    tab.addNumber("Aim velo (degrees / second)", () ->  (cancoder.getVelocity()).getValue());
    tab.addNumber("Aim amp", () -> (aimLaunchMotor.getStatorCurrent()).getValue());
    tab.addNumber("Aim volts", () -> (aimLaunchMotor.getMotorVoltage()).getValue());

    tab.addNumber("PID Want", () -> getPIDVoltage());
    tab.addNumber("FF want", () -> getFFVoltage());


    tab.add(this);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    ffWant = feedforward.calculate(setpoint.position, setpoint.velocity);

    aimLaunchMotor.setVoltage(output + ffWant);
  }
  public void startLauncher(double desiredVelocity) {
    rightLaunchMotor.set(desiredVelocity);
    leftLaunchMotor.set(desiredVelocity);
  }

  // public void startLauncher(LaunchCalculations launchcalculations) {
  // leftLaunchMotor.set(launchcalculations.getLaunchVelocity());
  // rightLaunchMotor.set(launchcalculations.getLaunchVelocity());
  // }

  public void stopLauncher() {
    leftLaunchMotor.set(0);
    rightLaunchMotor.set(0);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAimPosition();
  }

  public double getAimPosition() {
    return (cancoder.getPosition()).getValue() * (Math.PI / 180.0);
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

  public LaunchCalculations getLaunchCalculations(int id) {
    double vertDistance;
    double horizDistance;

    if (id == 6 || id == 5) {
      vertDistance = Constants.Launcher.ampHeight - Constants.Launcher.launcherHeight;
      horizDistance = AprilTag.getDirectDistance();
    } else if (id == 7 || id == 4) {
      vertDistance = Constants.Launcher.speakerHeight - Constants.Launcher.launcherHeight;
      horizDistance = AprilTag.getDirectDistance();
    } else if (id == 11 || id == 12 || id == 13 || id == 14 || id == 15 || id == 16) {
      vertDistance = Constants.Launcher.trapHeight - Constants.Launcher.launcherHeight;
      horizDistance = AprilTag.getDirectDistance();
    } else {
      vertDistance = 0;
      horizDistance = AprilTag.getDirectDistance();
    }
    return new LaunchCalculations(vertDistance, horizDistance);
  }

  public void aimLauncher(LaunchCalculations launchcalculations) {
    wantedAngle = launchcalculations.getLaunchAngle();
    if ((cancoder.getPosition()).getValue() < wantedAngle) {
      aimLaunchMotor.set(0.4);
    } else if ((cancoder.getPosition().getValue()) > wantedAngle) {
      aimLaunchMotor.set(-0.4);
    } else {
      aimLaunchMotor.set(0);
    }
  }

  public void lockInAim() {
    aimLaunchMotor.set(0);
  }
}
