package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Indexer extends SubsystemBase{
    private CANSparkMax firstIntakeMotor, secondIntakeMotor;

    public void indexer (){
        firstIntakeMotor = new CANSparkMax(0, MotorType.kBrushless);
        secondIntakeMotor = new CANSparkMax(0, MotorType.kBrushless);
        secondIntakeMotor.setInverted(true);
      }
    

    public void startIndexer(){
        firstIntakeMotor.set(0.5);
        secondIntakeMotor.set(0.5);
    }

    public void stopIndexer(){
        firstIntakeMotor.set(0);
        secondIntakeMotor.set(0);
    }
}