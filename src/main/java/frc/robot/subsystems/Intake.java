package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
    private CANSparkMax firstIntakeMotor, secondIntakeMotor;

    private boolean robotIntakeActivated = false;

    public Intake() {
        firstIntakeMotor = new CANSparkMax(0, MotorType.kBrushless);
        secondIntakeMotor = new CANSparkMax(0, MotorType.kBrushless);
        secondIntakeMotor.setInverted(true);
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
            firstIntakeMotor.set(0.5);
            secondIntakeMotor.set(0.5);
        } else {
            firstIntakeMotor.set(0);
            secondIntakeMotor.set(0);
        }
    }
}