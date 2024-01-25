package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    public CANSparkMax rightLaunchMotor, leftLaunchMotor;
    public CANEncoder rightMotorEncoder, leftMotorEncoder;
    public CANPIDController rightPIDController, leftPIDController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    public Launcher() {
        leftLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor.setInverted(true);

        leftMotorEncoder = leftLaunchMotor.getEncoder();
        rightMotorEncoder = rightLaunchMotor.getEncoder();

        leftPIDController = leftLaunchMotor.getPIDController();
        rightPIDController = rightLaunchMotor.getPIDController();

        leftPIDController.setFeedbackDevice(leftMotorEncoder);
        rightPIDController.setFeedbackDevice(rightMotorEncoder);

        kP = 6e-5;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.000015;
        kMaxOutput = 1;
        kMinOutput = -1;

        leftPIDController.setP(kP);
        leftPIDController.setI(kI);
        leftPIDController.setD(kD);
        leftPIDController.setIZone(kIz);
        leftPIDController.setFF(kFF);
        leftPIDController.setOutputRange(kMinOutput, kMaxOutput);

        rightPIDController.setP(kP);
        rightPIDController.setI(kI);
        rightPIDController.setD(kD);
        rightPIDController.setIZone(kIz);
        rightPIDController.setFF(kFF);
        rightPIDController.setOutputRange(kMinOutput, kMaxOutput);

        double setPoint = 0.0;
        rightPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
        leftPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    }
      

    public void launch() {
        double maxRPM = 5676;
    
        rightLaunchMotor.set();
    }
}
