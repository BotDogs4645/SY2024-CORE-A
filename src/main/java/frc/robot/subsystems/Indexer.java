package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    private PWMSparkMax IndexerMotor;

    public void indexer (){
        IndexerMotor = new PWMSparkMax(1);
    }

    public void startIndexer(double speed){
        IndexerMotor.set(speed);
    }

    public void stopIndexer(){
        IndexerMotor.set(0);
    }
}