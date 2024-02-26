package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.util.LaunchCalculations;

/**
 * The 'Launcher' class, pertaining to the use of FIRST Team 4645, The Chicago 
 * Style Bot Dogs' long-rang 'note' manipulation system for their 2024 Crescendo 
 * robot.
 */

public class Launcher extends SubsystemBase{

  private final LaunchCalculations launchCalculationsInstance;

  private TalonFX bottomLaunchMotor, topLaunchMotor;
  private CANSparkMax aimLaunchMotor;
  private CANcoder cancoder;
  private SparkPIDController controller;
  private double wantedAngle;

 /**
  * Acts as the constructor for the 'Launcher' class, through the declaration,
  * specification, and mapping of pertinent variables and instances of being
  * respectively.
  * 
  * @param launchCalculationsInstance - The 'LaunchCalculations' class instance
  * harnessed in the execution of this program, derived from the instance of 
  * such present within the method which called it.
  */ 
 public Launcher(LaunchCalculations launchCalculationsInstance) {
    this.launchCalculationsInstance = launchCalculationsInstance;
    topLaunchMotor = new TalonFX(13);
    bottomLaunchMotor = new TalonFX(14);
    aimLaunchMotor = new CANSparkMax(18, MotorType.kBrushless);
    controller = aimLaunchMotor.getPIDController();
    controller.setPositionPIDWrappingEnabled(false);
    controller.setPositionPIDWrappingMinInput(0);
    controller.setPositionPIDWrappingMaxInput(90);

  
    bottomLaunchMotor.setInverted(true);
    cancoder = new CANcoder(0);
  }

  /**
   * Activates the motors pertaining to the use of the robot's built-in 'note' 
   * launcher, through the setting of such to speeds specified in the method's 
   * parameters.
   * 
   * @param speed
   */
  public void startLauncher(double speed) {
    bottomLaunchMotor.set(speed);
    topLaunchMotor.set(speed);
  }

  /**
   * Deactivates the motors pertaining to the use of the robot's built-in 'note'
   * launcher, through the setting of such to a speed of 0, effectively halting
   * their movement.
   */
  public void stopLauncher() {
    topLaunchMotor.set(0);
    bottomLaunchMotor.set(0);
    aimLaunchMotor.set(0);
  }

  /**
   * Fetches the velocity at which the 'note' must be launched, if it is to
   * reach its intended target and act in a specific, pre-calculated manner, as
   * is required for such.
   * 
   * @return(s) the velocity at which the 'note' must be launched, if it is to
   * reach its intended target and act in a specific, pre-calculated manner, as 
   * is required for such.
   */
  public double getLaunchVelocity() {
    return launchCalculationsInstance.getLaunchVelocity();
  }

  /**
   * Specifies the angle at which the launcher must exist in order to achieve
   * the trajectory set forth by any given target.
   * 
   * @param wantedAngle - The angle at which the launcher should exist in order
   * to achieve a specific trajectory or result.
   */
  public void setWantedAngle(double wantedAngle) {
    this.wantedAngle = wantedAngle;
  }

  /**
   * Specifies the angle at which the launcher must exist in order to achieve
   * a specific trajectory or result, derived from the 'getLaunchAngle' method
   * within the 'LaunchCalculations' class.
   */
  public void aimLauncher() {
    wantedAngle = launchCalculationsInstance.getLaunchAngle();
    controller.setReference(wantedAngle, CANSparkMax.ControlType.kPosition);
  }
}