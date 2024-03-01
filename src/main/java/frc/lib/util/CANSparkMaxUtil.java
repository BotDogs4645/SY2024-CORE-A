package frc.lib.util;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

/** Sets motor usage for a Spark Max motor controller */
public enum CANSparkMaxUtil {
  kAll(20, 20, 50),
  kPositionOnly(500, 20, 500),
  kVelocityOnly(20, 500, 500),
  kMinimal(500, 500, 500);

  private final int status1, status2, status3;
  CANSparkMaxUtil(int status1, int status2, int status3) {
    this.status1 = status1;
    this.status2 = status2;
    this.status3 = status3;
  }


  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   */
  public void applyBusUsage(CANSparkMax motor) {
    applyBusUsage(motor, false);
  }

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   * @param enableFollowing Whether to enable motor following.
   */
  public void applyBusUsage(CANSparkMax motor, boolean enableFollowing) {
    int status0 = enableFollowing ? 10 : 500;

    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, status0);
    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, status1);
    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, status2);
    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, status3);
  }
}
