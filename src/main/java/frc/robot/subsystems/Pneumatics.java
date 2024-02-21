// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  DoubleSolenoid climber;
  DoubleSolenoid ampGuide;

  PneumaticsControlModule pcm;

  public Pneumatics() {
    climber = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      Constants.Pneumatics.climberForward,
      Constants.Pneumatics.climberReverse
    );

    ampGuide = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      Constants.Pneumatics.ampGuideForward,
      Constants.Pneumatics.ampGuideReverse
    );

    pcm = new PneumaticsControlModule(Constants.Pneumatics.pcmCanID);
  }

  public void extendClimber() {
    climber.set(DoubleSolenoid.Value.kForward);
  }
  public void retractClimber() {
    climber.set(DoubleSolenoid.Value.kReverse);
  }

  public void extendAmpGuide() {
    ampGuide.set(DoubleSolenoid.Value.kForward);
  }
  public void retractAmpGuide() {
    ampGuide.set(DoubleSolenoid.Value.kReverse);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
