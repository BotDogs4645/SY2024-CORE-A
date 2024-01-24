package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public TalonFX leftMotor;
    public TalonFX rightMotor;
    public CANcoder cancoder;

    public Arm() {
        leftMotor = new TalonFX(14);
        rightMotor = new TalonFX(15);
        cancoder = new CANcoder(16);
        rightMotor.setInverted(true);
    }

    StatusSignal<Double> rotations;
    double degrees; 
    

    public void moveArm(int id) {
        rotations = cancoder.getPosition();
        degrees = rotations.getValue() * 360.0;
        if (id == 1 && degrees < 45) {
            leftMotor.set(0.4);
            rightMotor.set(0.4);

        }else if (id == 2 && degrees < 90) {
            leftMotor.set(0.4);
            rightMotor.set(0.4);
        } else {
            leftMotor.set(0);
            rightMotor.set(0);
            leftMotor.stopMotor();
            rightMotor.stopMotor();
         }

    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
    }

}
