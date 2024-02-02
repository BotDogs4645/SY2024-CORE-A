package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Indexer {
private CANSparkMax leftLaunchMotor;
private CANSparkMax rightLaunchMotor;
//intake motors exist
    public void indexer (){
        leftLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor.setInverted(true);
        //intake motors gain conciousness

        leftLaunchMotor.set(0.4);
        rightLaunchMotor.set(0.4);

        //intake motors go brrr

        

        long past = System.currentTimeMillis();
        if (System.currentTimeMillis() - past >= 1){
          leftLaunchMotor.set(0);
          rightLaunchMotor.set(0);
         //intake motors stop going brrr
      }
    }
}

