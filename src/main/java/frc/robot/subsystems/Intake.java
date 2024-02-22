package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Intake extends SubsystemBase {
    private PWMSparkMax IntakeMotor;
    private boolean robotIntakeActivated = false;

    public Intake() {
        IntakeMotor = new PWMSparkMax(0);
    }

    public void toggleIntake() {
        robotIntakeActivated = !robotIntakeActivated;
        dynamicIntakeExecution(robotIntakeActivated);
    }

    public void setIntake(boolean activated) {
        robotIntakeActivated = activated;
        dynamicIntakeExecution(robotIntakeActivated);
    }

    public void dynamicIntakeExecution(boolean robotIntakeActivated) {
        if (robotIntakeActivated) {
            IntakeMotor.set(Constants.Intake.intakeSpeed);
        } else {
            IntakeMotor.set(0);
        }
    }
}