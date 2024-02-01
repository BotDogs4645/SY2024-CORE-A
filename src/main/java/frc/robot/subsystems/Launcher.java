package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LaunchCalculations;

import frc.robot.Constants;

public class Launcher extends ProfiledPIDSubsystem {
    private CANSparkMax rightLaunchMotor, leftLaunchMotor, aimLaunchMotor;
    private RelativeEncoder rightMotorEncoder, leftMotorEncoder, aimMotorEncoder;
    private PIDController rightPIDController, leftPIDController, aimPIDController;
    //all the rotational PID stuff

    private ArmFeedforward feedforward;
    private double pidWant;
    private double ffWant;
    public double wantedAngle;
    private ShuffleboardTab tab;
    // more stuff for launch calculations
    double vertDistance = 0;
    double horizDistance = 0;

    public Launcher() {
            super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            8.25,
            0,
            0.05,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1.5, 0.7)));

    this.feedforward = new ArmFeedforward(0.48005, 0.54473, 1.3389, 0.19963);
    this.ffWant = 0;
    super.getController().setTolerance(1.5, .5);
        leftLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        aimLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor.setInverted(true);

        leftMotorEncoder = leftLaunchMotor.getEncoder();
        rightMotorEncoder = rightLaunchMotor.getEncoder();
        aimMotorEncoder = aimLaunchMotor.getEncoder();

        leftPIDController = new PIDController(Constants.Launcher.kP, Constants.Launcher.kI, Constants.Launcher.kD);
        rightPIDController = new PIDController(Constants.Launcher.kP, Constants.Launcher.kI, Constants.Launcher.kD);
        aimPIDController = new PIDController(Constants.Launcher.kP, Constants.Launcher.kI, Constants.Launcher.kD);

        this.tab = Shuffleboard.getTab("Launcher");
        tab.addNumber("Launcher Angle (degrees)", () -> aimMotorEncoder.getPosition());
        tab.addNumber("PID Want", () -> getPIDVoltage());
        tab.addNumber("FF want", () -> getFFVoltage());


        tab.add(this);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    ffWant = feedforward.calculate(setpoint.position, setpoint.velocity);

    aimLaunchMotor.setVoltage(output + ffWant);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAimPosition();
  }

  public double getAimPosition() {
    return  aimMotorEncoder.getPosition() * (Math.PI / 180.0);
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


    public void startLauncher(double desiredVelocity) {
        rightLaunchMotor.set(rightPIDController.calculate(rightMotorEncoder.getVelocity(), desiredVelocity));
        leftLaunchMotor.set(leftPIDController.calculate(leftMotorEncoder.getVelocity(), desiredVelocity));
        
    }

    public void aimLauncher(int id){
        
        if (id == 6 || id == 5){
            vertDistance = Constants.Launcher.ampHeight - Constants.Launcher.launcherHeight;
            horizDistance = robot.getDirectDistance();
        }
        else if (id == 7 || id == 4){
            vertDistance = Constants.Launcher.speakerHeight - Constants.Launcher.launcherHeight;
            horizDistance = robot.getDirectDistance();
        }
        else if (id == 11 || id == 12 || id == 13 || id == 14 || id == 15 || id == 16){
            vertDistance = Constants.Launcher.trapHeight - Constants.Launcher.launcherHeight;
            horizDistance = limelight.getDirectDistance();
        }
        else{
            vertDistance = 0;
            horizDistance = limelight.getDirectDistance();
        }
        LaunchCalculations launchcalculations = new LaunchCalculations(vertDistance, horizDistance);
        wantedAngle = launchcalculations.getLaunchAngle();
        if (aimMotorEncoder.getPosition() < wantedAngle){
            aimLaunchMotor.set(0.4);
        }
        else if (aimMotorEncoder.getPosition() > wantedAngle){
            aimLaunchMotor.set(-0.4);
            }
            else {
                aimLaunchMotor.set(0);
            }
        }
    public void launchNote(int id) throws InterruptedException{
        LaunchCalculations launchcalculations = new LaunchCalculations(vertDistance, horizDistance);
        leftLaunchMotor.set(launchcalculations.getLaunchVelocity());
        rightLaunchMotor.set(launchcalculations.getLaunchVelocity());
        wait(2000);
        leftLaunchMotor.set(0);
        rightLaunchMotor.set(0);
    }
        
    }

