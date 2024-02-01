package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LaunchCalculations;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
    private CANSparkMax rightLaunchMotor, leftLaunchMotor;
    private RelativeEncoder rightMotorEncoder, leftMotorEncoder;
    private PIDController rightPIDController, leftPIDController;


    public Launcher() {
        leftLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor.setInverted(true);

        leftMotorEncoder = leftLaunchMotor.getEncoder();
        rightMotorEncoder = rightLaunchMotor.getEncoder();

        leftPIDController = new PIDController(Constants.Launcher.kP, Constants.Launcher.kI, Constants.Launcher.kD);
        rightPIDController = new PIDController(Constants.Launcher.kP, Constants.Launcher.kI, Constants.Launcher.kD);
    }

    public void startLauncher(double desiredVelocity) {
        rightLaunchMotor.set(rightPIDController.calculate(rightMotorEncoder.getVelocity(), desiredVelocity));
        leftLaunchMotor.set(leftPIDController.calculate(leftMotorEncoder.getVelocity(), desiredVelocity));   
    }
}