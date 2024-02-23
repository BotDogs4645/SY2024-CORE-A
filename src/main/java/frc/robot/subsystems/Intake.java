package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

/**
 * The 'Intake' class, pertaining to the manipulation and acquisition of 'notes'
 * within FIRST Robotics' 2024 Crescendo competition, created specifically for
 * the use of Team 4645, The Chicago Style Bot Dogs.
 */

public class Intake extends SubsystemBase {

    private CANSparkMax firstIntakeMotor, secondIntakeMotor;

    public boolean intakeEnabled;

    /**
     * Acts as the constructor for the 'Intake' class, through the use of 
     * variable declaration, parameter specification, and more.
     */
    public Intake() {
        firstIntakeMotor = new CANSparkMax(17, MotorType.kBrushless);
        secondIntakeMotor = new CANSparkMax(18, MotorType.kBrushless);
        secondIntakeMotor.setInverted(true);

        intakeEnabled = false;
    }

    /**
     * Activates the Intake's motors at a rate specified within the method's 
     * parameters.
     * 
     * @param intakeSpeed - The speed at which the intake motors should rotate.
     */
    public void setSpecificIntakeSpeed(double intakeSpeed) {
        intakeEnabled = true;
        
        firstIntakeMotor.set(intakeSpeed);
        secondIntakeMotor.set(intakeSpeed);
    }

    /**
     * Begins the intake's execution through the setting of both motors to a
     * rate specified in 'Constants.java.'
     */    
    public void activateIntake() {
        intakeEnabled = true;

        firstIntakeMotor.set(Constants.Intake.defaultMotorSpeed);
        secondIntakeMotor.set(Constants.Intake.defaultMotorSpeed);
    }

    /**
     * Halts the intake's execution through the setting of both of its
     * associated motors to a speed of 0.
     */
    public void deactivateIntake() {
        intakeEnabled = false;

        firstIntakeMotor.set(0);
        secondIntakeMotor.set(0);
    }
}
