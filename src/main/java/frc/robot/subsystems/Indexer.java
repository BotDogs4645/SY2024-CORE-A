package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Indexer {
private CANSparkMax leftLaunchMotor;
private CANSparkMax rightLaunchMotor;
//intake motors exist
    public void indexer () throws InterruptedException{
        leftLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightLaunchMotor.setInverted(true);
        //intake motors gain conciousness

        leftLaunchMotor.set(0.4);
        rightLaunchMotor.set(0.4);

        //intake motors go brrr

        wait(1000);

        leftLaunchMotor.set(0);
        rightLaunchMotor.set(0);
        //intake motors stop going brrr
    }
}
