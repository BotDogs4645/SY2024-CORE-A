package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  private final int climberForwardChannel = 4;
  private final int climberReverseChannel = 5;



  private DoubleSolenoid climberDoubleSolenoid;

  private Compressor compressor;

  public Pneumatics() {
    
    climberDoubleSolenoid = new DoubleSolenoid(Constants.pcmCanID, PneumaticsModuleType.CTREPCM, climberForwardChannel, climberReverseChannel);

    compressor = new Compressor(Constants.pcmCanID, PneumaticsModuleType.CTREPCM);

    climberDoubleSolenoid.set(kReverse);
    compressor.getPressureSwitchValue();
  }

  public Command retractClimber() {
    return runOnce(
        () -> {
          climberDoubleSolenoid.set(kReverse);
        });
  }

  public Command extendClimber() {
    return runOnce(
        () -> {
          climberDoubleSolenoid.set(kForward);
        });
  }

  public Command toggleClimber() {
    return runOnce(
        () -> {
          climberDoubleSolenoid.toggle();
        });
  }

  public boolean pressureSwitchStatus() {
    return compressor.getPressureSwitchValue();
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {}
}
