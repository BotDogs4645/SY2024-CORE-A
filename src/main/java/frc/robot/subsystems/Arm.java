package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.TalonFX;
import com.ctre.phoenix.sensors.Cancoder;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private TalonFX leftMotor;
    private TalonFX rightMotor;
    int degrees = _coder.getPosition();
    leftMotor = new TalonFX(LEFT_MOTOR, MotorType.kBrushless);
    rightMotor = new TalonFX(RIGHT_MOTOR, MotorType.kBrushless);
    rightMotor.setInverted(true);



    public Arm() {
    if (tid == 1) {
        while (degrees < 45){
            leftMotor.set(0.5);
            rightMotor.set(0.5);
        }
        leftMotor.set(0);
        rightMotor.set(0);
    }
    else if (tid == 2){
        while (degrees < 90){
            leftMotor.set(0.5);
            rightMotor.set(0.5);
        }
        leftMotor.set(0);
        rightMotor.set(0);
    }
    else{
        leftMotor.set(0);
        rightMotor.set(0);
    }

       

}
}

  @Override
  public void periodic() {
    
  }
  @Override
  public void simulationPeriodic() {}
}
