package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class Indexer extends SubsystemBase{
    private CANSparkMax firstIntakeMotor, secondIntakeMotor;
    private PIDController firstPIDController, secondPIDController;
    private RelativeEncoder firstMotorEncoder, secondMotorEncoder;
    public void indexer (){
        firstIntakeMotor = new CANSparkMax(0, MotorType.kBrushless);
        secondIntakeMotor = new CANSparkMax(0, MotorType.kBrushless);
        secondIntakeMotor.setInverted(true);

        firstPIDController = new PIDController(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD);
        secondPIDController = new PIDController(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD);

        firstMotorEncoder = firstIntakeMotor.getEncoder();
        secondMotorEncoder = secondIntakeMotor.getEncoder();

      }
    

    public void indexing(double speed){

        firstIntakeMotor.set(0.5);
        secondIntakeMotor.set(0.5);
    }

    public void pleaseStop(){
        firstIntakeMotor.set(0);
        secondIntakeMotor.set(0);
    }
}
