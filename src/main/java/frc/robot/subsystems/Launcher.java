package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
    private CANSparkMax rightLaunchMotor, leftLaunchMotor;
    private RelativeEncoder rightMotorEncoder, leftMotorEncoder;
    private PIDController rightPIDController, leftPIDController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    public Launcher() {
        leftLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor.setInverted(true);

        leftMotorEncoder = leftLaunchMotor.getEncoder();
        rightMotorEncoder = rightLaunchMotor.getEncoder();

        kP = 6e-5;
        kI = 0;
        kIz = 0;
        //implement kD
        kFF = 0.000015;
        kMaxOutput = 1;
        kMinOutput = -1;

        leftPIDController = new PIDController(kP, kI, kD);
        rightPIDController = new PIDController(kP, kI, kD);

    }

    public void startLauncher(double desiredVelocity) {
        // double percentOutput = desiredVelocity/ Constants.Swerve.maxSpeed;
        // leftLaunchMotor.set(percentOutput);
        // rightLaunchMotor.set(percentOutput);

        rightLaunchMotor.set(rightPIDController.calculate(rightMotorEncoder.getVelocity(), desiredVelocity));
        leftLaunchMotor.set(leftPIDController.calculate(leftMotorEncoder.getVelocity(), desiredVelocity));
        
    }

    public double getLaunchVelocity(double verticalDistance, double horizontalDistance) {
        double verticalVelocity = Math.sqrt(2 * Constants.Launcher.gravityAcceleration * verticalDistance);
        double timeToTravel = (-1 * verticalVelocity + Math.sqrt(Math.pow(verticalVelocity, 2) + 2 * Constants.Launcher.gravityAcceleration * verticalDistance))/(Constants.Launcher.gravityAcceleration);
        double horizontalVelocity = horizontalDistance/timeToTravel;
        double launchVelocity = Math.sqrt(Math.pow(verticalVelocity, 2) + Math.pow(horizontalVelocity, 2));
        return launchVelocity;
    }
}
