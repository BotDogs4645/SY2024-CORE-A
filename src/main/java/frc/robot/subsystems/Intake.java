package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private TalonFX firstIntakeMotor, secondIntakeMotor;

    public boolean intakeEnabled;

    public Intake() {
        firstIntakeMotor = new TalonFX(17);
        secondIntakeMotor = new TalonFX(18);
        secondIntakeMotor.setInverted(true);

        intakeEnabled = false;
    }

    // Allows the program to set a specific speed at which the intake will run.

    public void setSpecificIntakeSpeed(double intakeSpeed) {
        intakeEnabled = true;
        
        firstIntakeMotor.set(intakeSpeed);
        secondIntakeMotor.set(intakeSpeed);
    }

    // Initates the intake through the activation of each motor at the speed defined in in the 'Constants.Intake' class.
    
    public void activateIntake() {
        intakeEnabled = true;

        firstIntakeMotor.set(Constants.Intake.defaultMotorSpeed);
        secondIntakeMotor.set(Constants.Intake.defaultMotorSpeed);
    }

    // Halts the intake through the deactivation of each of the associated motors.

    public void deactivateIntake() {
        intakeEnabled = false;

        firstIntakeMotor.set(0);
        secondIntakeMotor.set(0);
    }
}
