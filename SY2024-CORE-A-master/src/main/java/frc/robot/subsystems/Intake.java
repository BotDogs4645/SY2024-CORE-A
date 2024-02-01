package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;

import javax.xml.crypto.dsig.XMLObject;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax firstIntakeMotor, secondIntakeMotor;
    private PIDController firstPIDController, secondPIDController;
    private RelativeEncoder firstMotorEncoder, secondMotorEncoder;

    private static boolean robotIntakeActivated;
    private static boolean intakeSequenceActivated;

    public Intake() {
        firstIntakeMotor = new CANSparkMax(0, MotorType.kBrushless);
        secondIntakeMotor = new CANSparkMax(0, MotorType.kBrushless);
        secondIntakeMotor.setInverted(true);

        firstPIDController = new PIDController(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD);
        secondPIDController = new PIDController(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD);

        firstMotorEncoder = firstIntakeMotor.getEncoder();
        secondMotorEncoder = secondIntakeMotor.getEncoder();
    }

    public static void intakePeriodic() {
        robotIntakeActivated = RobotContainer.driveController.leftTrigger().getAsBoolean();
        if (robotIntakeActivated == false && intakeSequenceActivated == true) {
            intakeSequenceActivated = false;
        } else if (robotIntakeActivated == true && intakeSequenceActivated == false) {
            intakeSequenceActivated = true;
        }
    }

    public void beginExecution() {
        if (intakeSequenceActivated == true) {
            firstIntakeMotor.set(firstPIDController.calculate(firstMotorEncoder.getVelocity(), Constants.Intake.intakeMotorVelocity));
            secondIntakeMotor.set(secondPIDController.calculate(secondMotorEncoder.getVelocity(), Constants.Intake.intakeMotorVelocity));
        } else {
            firstIntakeMotor.set(0);
            secondIntakeMotor.set(0);
        }
    }
}
