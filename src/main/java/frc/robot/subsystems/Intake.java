package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax firstIntakeMotor, secondIntakeMotor;
    private PIDController firstPIDController, secondPIDController;
    private RelativeEncoder firstMotorEncoder, secondMotorEncoder;

    private boolean robotIntakeActivated = false;

    public Intake() {
        firstIntakeMotor = new CANSparkMax(0, MotorType.kBrushless);
        secondIntakeMotor = new CANSparkMax(0, MotorType.kBrushless);
        secondIntakeMotor.setInverted(true);

        firstPIDController = new PIDController(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD);
        secondPIDController = new PIDController(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD);

        firstMotorEncoder = firstIntakeMotor.getEncoder();
        secondMotorEncoder = secondIntakeMotor.getEncoder();
    }

    public void toggleIntake() {
        robotIntakeActivated = !robotIntakeActivated;
        dynamicIntakeExecution(robotIntakeActivated);
    }

    public void setIntake(boolean activated) {
        robotIntakeActivated = activated;
    }

    public void dynamicIntakeExecution(boolean robotIntakeActivated) {
        if (robotIntakeActivated) {
            firstIntakeMotor.set(firstPIDController.calculate(firstMotorEncoder.getVelocity(), Constants.Intake.intakeMotorVelocity));
            secondIntakeMotor.set(secondPIDController.calculate(secondMotorEncoder.getVelocity(), Constants.Intake.intakeMotorVelocity));
        } else {
            firstIntakeMotor.set(0);
            secondIntakeMotor.set(0);
        }
    }
}