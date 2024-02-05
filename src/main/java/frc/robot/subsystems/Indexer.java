package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;


public class Indexer extends SubsystemBase{
private CANSparkMax leftLaunchMotor;
private CANSparkMax rightLaunchMotor;
//intake motors exist
    public void indexer (){
        leftLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor.setInverted(true);
        //intake motors gain conciousness

      }
    

    public void indexing(double speed){
        leftLaunchMotor.set(speed);
        rightLaunchMotor.set(speed);
        //intake motors go brrr
    }

    public void pleaseStop(){
          leftLaunchMotor.set(0);
          rightLaunchMotor.set(0);
         //intake motors stop going brrr
    }
}
