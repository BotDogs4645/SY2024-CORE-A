package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private CANSparkMax firstIntakeMotor, secondIntakeMotor;

    public boolean intakeEnabled;

    public Intake() {
        firstIntakeMotor = new CANSparkMax(17, MotorType.kBrushless);
        secondIntakeMotor = new CANSparkMax(18, MotorType.kBrushless);
        secondIntakeMotor.setInverted(true);

        intakeEnabled = false;
    }

    // Allows the program to set a specific speed at which the intake will run.

    public void setSpecificIntakeSpeed(double intakeSpeed) {
        intakeEnabled = true;
        
        firstIntakeMotor.set(intakeSpeed);
        secondIntakeMotor.set(intakeSpeed);
    }

    // Activates the intake through the activation of each motor at the speed defined in in the 'Constants.Intake' class.
    
    public void activateIntake() {
        intakeEnabled = true;

        firstIntakeMotor.set(Constants.Intake.defaultMotorSpeed);
        secondIntakeMotor.set(Constants.Intake.defaultMotorSpeed);
    }

    // Deactivates the intake through the deactivation of each of the associated motors.

    public void deactivateIntake() {
        intakeEnabled = false;

        firstIntakeMotor.set(0);
        secondIntakeMotor.set(0);
    }
}
